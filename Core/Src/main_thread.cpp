#include <cstdio>
#include <ctime>

#include <halx/core.hpp>
#include <halx/rtos.hpp>
#include <halx/peripheral.hpp>
#include <halx/driver/c6x0.hpp>

#include "bno055_double.hpp"
#include "lowpass.hpp"
#include "motor_free_rev.hpp"
#include "output_to_logger.hpp"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
///////////////

///////
extern "C" void main_thread(void *) {
  using namespace halx::peripheral;
  using namespace halx::rtos;
  using namespace halx::driver;

  Uart<&huart2> uart2; // デバッグ出力用
  enable_stdout(uart2);

  // I2C i2c_28(&hi2c1, 0x29); // BNO055のI2Cアドレスは0x28
  // I2C i2c_29(&hi2c1, 0x28); // BNO055のI2Cアドレスは0x28
  // Bno055 bno055_28(i2c_28);
  // Bno055 bno055_29(i2c_29);

  LowPassFilter lowpass_filter_1(20);
  LowPassFilter lowpass_filter_2(20);
  LowPassFilter lowpass_filter_vel_1(10);
  LowPassFilter lowpass_filter_vel_2(10);
  LowPassFilter lowpass_filter_imu(20);

  Can<&hfdcan1> can;
  C6x0Manager c610_manager(can);
  C6x0 c610_1(c610_manager, C6x0Type::C610, C6x0Id::ID_1); // front motor
  C6x0 c610_2(c610_manager, C6x0Type::C610, C6x0Id::ID_2); // rear motor
  can.start();

  MotorFreeRev motor_free_rev_1(c610_1, 2.0f);
  MotorFreeRev motor_free_rev_2(c610_2, 2.0f);

  float a1;
  float a2;
  float b1;
  float b2;

  float st;
  float th;

  float motor_acc_1;
  float motor_acc_2;
  static float prevous_motor_rps_1;
  static float prevous_motor_rps_2;

  static float prevous_time;

  int motor_control_singnnal_1;
  int motor_control_singnnal_2;

  float steer_angle;
  float desire_yaw;
  std::vector<float> csvrow;

  float motor_set_current_1;
  float motor_set_current_2;
  float control_boarder = 0.1;

  float yaw_control_Kp = 4000.0f;
  float yaw_control_Ki = 40.0f;
  float error_yaw_sum = 0.0f;

  // Quaternion quat_28, quat_29;
  // AngularVelocity ang_vel_28, ang_vel_29;
  // Acceleration acc_28, acc_29;

  // スレッドの外で定義
  Bno055_double bno055_instance(&hi2c1); // 適切な引数で初期化
  Bno055Data bno055_data_instance;

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  while (!bno055_instance.start(1000)) {
    printf("Failed to initialize BNO055. Retrying...\n");
    halx::core::delay(100);
  }

  ////////////////
  Thread bno055_thread(
      [&bno055_instance, &bno055_data_instance]() {
        while (true) {
          bno055_instance.get_bno(bno055_data_instance);
          halx::core::delay(10);
        }
      },
      2048, osPriorityNormal);
  // ///////////
  while (true) {
    uint32_t start = halx::core::get_tick();

    a1 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    a2 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
    st = ((a1 / a2) - 0.8914) / 0.0257;
    steer_angle = st * 17.0f * 3.14f / 180.0f;

    b1 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    b2 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    th = ((b1 / (b2)) - 0.89175) / (0.03175);

    c610_manager.update();

    float motor_vel_1 = c610_1.get_rpm() / 60.0f;
    float motor_vel_2 = c610_2.get_rpm() / 60.0f;

    motor_acc_1 = (motor_vel_1 * 0.0410311 - prevous_motor_rps_1) * 1000 /
                  (halx::core::get_tick() - prevous_time);
    // 回転数×車輛速度のための係数×ミリ秒
    motor_acc_2 = (motor_vel_2 * 0.0410311 - prevous_motor_rps_2) * 1000 /
                  (halx::core::get_tick() - prevous_time);

    // float filtered_motor_acc_1 =
    //     lowpass_filter_1.lowpass(motor_acc_1, halx::core::get_tick());
    // float filtered_motor_acc_2 =
    //     lowpass_filter_2.lowpass(motor_acc_2, halx::core::get_tick());

    prevous_time = halx::core::get_tick();

    prevous_motor_rps_1 = motor_vel_1 * 0.0410311;
    prevous_motor_rps_2 = motor_vel_2 * 0.0410311;

    motor_set_current_1 = -1 * th * 9000;
    motor_set_current_2 = th * 9000;

    // motor_free_rev_1.free_rev(motor_set_current_1);
    // motor_free_rev_2.free_rev(motor_set_current_2);

    c610_1.set_current_ref(motor_set_current_1); // マイナス前進
    c610_2.set_current_ref(motor_set_current_2); // プラス前進
    c610_manager.transmit();

    // ログ出力
    LoggerCsvRow row;
    row.tick = halx::core::get_tick();
    row.get_acc_x_28 = bno055_data_instance.acc_28.x;
    row.get_acc_y_28 = bno055_data_instance.acc_28.y;
    row.get_gyr_z_28 = bno055_data_instance.ang_vel_28.z;
    row.get_acc_x_29 = bno055_data_instance.acc_29.x;
    row.get_acc_y_29 = bno055_data_instance.acc_29.y;
    row.get_gyr_z_29 = bno055_data_instance.ang_vel_29.z;
    row.get_motor_acc_1 = motor_acc_1;
    row.get_motor_acc_2 = motor_acc_2;
    row.get_prevous_motor_rps_1 = prevous_motor_rps_1;
    row.get_prevous_motor_rps_2 = prevous_motor_rps_2;
    row.get_steer_angle = steer_angle;
    row.get_motor_set_current_1 = motor_set_current_1;
    row.get_motor_set_current_2 = motor_set_current_2;

    printf("%s\n", row.format());
    halx::core::delay_until(start + 10);
  }
}
