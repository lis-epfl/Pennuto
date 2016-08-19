/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file main.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief Main file
 *
 ******************************************************************************/

#include "sample_projects/Pennuto/pennuto.hpp"

#include "boards/megafly_rev4/megafly_rev4.hpp"

// #include "hal/dummy/file_dummy.hpp"
#include "hal/avr32/file_flash_avr32.hpp"
#include "hal/avr32/serial_usb_avr32.hpp"

// //uncomment to go in simulation
// #include "simulation/dynamic_model_quad_diag.hpp"
// #include "simulation/simulation.hpp"
// #include "hal/dummy/adc_dummy.hpp"
// #include "hal/dummy/pwm_dummy.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/piezo_speaker.h"
#include "libs/asf/avr32/services/delay/delay.h"

#include "sample_projects/Pennuto/proj_avr32/config/conf_imu.hpp"
}

// #include "hal/common/dbg.hpp"

int main(void)
{
    bool init_success = true;

    // -------------------------------------------------------------------------
    // Create board
    // -------------------------------------------------------------------------
    megafly_rev4_conf_t board_config    = megafly_rev4_default_config();
    board_config.imu_config             = imu_config();                         // Load custom imu config (cf conf_imu.h)
    /*On Pennuto platform the autopilot is upside down
     so we need to adapt the axis signs for Y and Z
    */
    board_config.imu_config.accelerometer.sign[1] *= -1.0f;
    board_config.imu_config.accelerometer.sign[2] *= -1.0f;
    board_config.imu_config.gyroscope.sign[1] *= -1.0f;
    board_config.imu_config.gyroscope.sign[2] *= -1.0f;
    board_config.imu_config.magnetometer.sign[1] *= -1.0f;
    board_config.imu_config.magnetometer.sign[2] *= -1.0f;
    /*On Pennuto platform the servos are connected as follow :
    servo_wing_left -> index 0
    servo_tail      -> index 1
    motor           -> index 5 ( ESC )
    servo_elevator  -> index 6
    servo_wing_right-> index 7
    */
    board_config.servo_config[0]    = servo_default_config_standard();
    board_config.servo_config[1]    = servo_default_config_standard();
    board_config.servo_config[6]    = servo_default_config_standard();
    board_config.servo_config[7]    = servo_default_config_standard();
    Megafly_rev4 board = Megafly_rev4(board_config);

    // Board initialisation
    init_success &= board.init();

    fat_fs_mounting_t fat_fs_mounting;

    fat_fs_mounting_init(&fat_fs_mounting);

    File_fat_fs file_log(true, &fat_fs_mounting); // boolean value = debug mode
    File_fat_fs file_stat(true, &fat_fs_mounting); // boolean value = debug mode

    // -------------------------------------------------------------------------
    // Create MAV
    // -------------------------------------------------------------------------
    // Create MAV using real sensors
    Pennuto::conf_t mav_config = Pennuto::default_config(MAVLINK_SYS_ID);
    Pennuto mav = Pennuto(board.imu,
                        board.bmp085,
                        board.gps_ublox,
                        board.sonar_i2cxl,      // Warning:
                        board.uart0,
                        board.spektrum_satellite,
                        board.state_display_megafly_rev4_,
                        board.file_flash,
                        board.battery,
                        board.servo_0,
                        board.servo_1,
                        board.servo_2,
                        board.servo_3,
                        board.servo_4,
                        board.servo_5,
                        board.servo_6,
                        board.servo_7,
                        file_log,
                        file_stat,
                        mav_config );

    // // -------------------------------------------------------------------------
    // // Create simulation
    // // -------------------------------------------------------------------------

    // // // Simulated servos
    // Pwm_dummy pwm[8];
    // Servo sim_servo_0(pwm[0], servo_default_config_esc());
    // Servo sim_servo_1(pwm[1], servo_default_config_esc());
    // Servo sim_servo_2(pwm[2], servo_default_config_esc());
    // Servo sim_servo_3(pwm[3], servo_default_config_esc());
    // Servo sim_servo_4(pwm[4], servo_default_config_esc());
    // Servo sim_servo_5(pwm[5], servo_default_config_esc());
    // Servo sim_servo_6(pwm[6], servo_default_config_esc());
    // Servo sim_servo_7(pwm[7], servo_default_config_esc());

    // // Create MAV using simulated sensors
    // Pennuto::conf_t mav_config = Pennuto::default_config(MAVLINK_SYS_ID);

    // // Simulated dynamic model
    // Dynamic_model_quad_diag sim_model    = Dynamic_model_quad_diag(sim_servo_0, sim_servo_1, sim_servo_2, sim_servo_3);
    // Simulation sim                       = Simulation(sim_model);

    // // Simulated battery
    // Adc_dummy    sim_adc_battery = Adc_dummy(11.1f);
    // Battery  sim_battery     = Battery(sim_adc_battery);

    // // Simulated IMU
    // Imu      sim_imu         = Imu(  sim.accelerometer(),
    //                                  sim.gyroscope(),
    //                                  sim.magnetometer() );

    // // set the flag to simulation
    // Pennuto mav = Pennuto( sim_imu,
    //                      sim.barometer(),
    //                      sim.gps(),
    //                      sim.sonar(),
    //                      board.uart0,                // mavlink serial
    //                      board.spektrum_satellite,
    //                      board.state_display_megafly_rev4_,
    //                      board.file_flash,
    //                      sim_battery,
    //                      sim_servo_0,
    //                      sim_servo_1,
    //                      sim_servo_2,
    //                      sim_servo_3 ,
    //                      sim_servo_4,
    //                      sim_servo_5,
    //                      sim_servo_6,
    //                      sim_servo_7 ,
    //                      file_log,
    //                      file_stat,
    //                      mav_config );

    if (init_success)
    {
        piezo_speaker_quick_startup();

        // Switch off red LED
        board.red_led.off();
    }
    else
    {
        piezo_speaker_critical_error_melody();
    }

    print_util_dbg_print("[MAIN] OK. Starting up.\r\n");

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------
    mav.loop();

    return 0;
}
