/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names
 * of its contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH CORPORATION BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file kore.cpp
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The main source file for the Krang support file which has the
 * definition for the Hardware struct constructor.
 */

#include "kore.hpp"
#include <unistd.h>
#include "kore/display.hpp"

using namespace Eigen;

namespace Krang {

/* ********************************************************************************************
 */
Hardware::Hardware(Mode mode_, somatic_d_t* daemon_cx_,
                   dart::dynamics::SkeletonPtr robot_, bool filter_imu) {
  // Set the local variables
  daemon_cx = daemon_cx_;
  robot = robot_;
  mode = mode_;

  // Initialize all the 'optional' pointers to nulls and sanity check the inputs
  amc = torso = NULL;
  //	fts[LEFT] = fts[RIGHT] = NULL;
  arms[LEFT] = arms[RIGHT] = NULL;
  grippers[LEFT] = grippers[RIGHT] = NULL;
  waistCmdChan = NULL;
  assert((daemon_cx != NULL) &&
         "Can not give null daemon context to hardware constructor");
  assert((robot != NULL) &&
         "Can not give null dart kinematics to hardware constructor");

  // Initialize the imu sensor and average the first 500 values
  initImu(filter_imu);

  // Define the pos/vel limits for the motor groups
  VectorXd lim7 = VectorXd::Ones(7) * 1024.1;
  VectorXd lim4 = VectorXd::Ones(4) * 1024.1;
  VectorXd lim2 = VectorXd::Ones(2) * 1024.1;
  VectorXd lim1 = VectorXd::Ones(1) * 1024.1;

  // Initialize the Schunk (+ Robotiq) motor groups
  if (mode & MODE_LARM)
    initMotorGroup(daemon_cx, arms[LEFT], "llwa-cmd", "llwa-state", -lim7, lim7,
                   -lim7, lim7);
  if (mode & MODE_RARM)
    initMotorGroup(daemon_cx, arms[RIGHT], "rlwa-cmd", "rlwa-state", -lim7,
                   lim7, -lim7, lim7);
  if (mode & MODE_TORSO)
    initMotorGroup(daemon_cx, torso, "torso-cmd", "torso-state", -lim1, lim1,
                   -lim1, lim1);
  if ((mode & MODE_LARM) && (mode & MODE_GRIPPERS_SCH))
    initMotorGroup(daemon_cx, grippers[LEFT], "lgripper-cmd", "lgripper-state",
                   -lim1, lim1, -lim1, lim1);
  if ((mode & MODE_LARM) && (mode & MODE_GRIPPERS))
    initMotorGroup(daemon_cx, grippers[LEFT], "lgripper-cmd", "lgripper-state",
                   -lim4, lim4, -lim4, lim4);
  if ((mode & MODE_RARM) && (mode & MODE_GRIPPERS_SCH))
    initMotorGroup(daemon_cx, grippers[RIGHT], "rgripper-cmd", "rgripper-state",
                   -lim1, lim1, -lim1, lim1);
  if ((mode & MODE_RARM) && (mode & MODE_GRIPPERS))
    initMotorGroup(daemon_cx, grippers[RIGHT], "rgripper-cmd", "rgripper-state",
                   -lim4, lim4, -lim4, lim4);

  // Initialize the wheel motor groups which depend on imu readings to get
  // absolute wheel positions
  if (mode & MODE_AMC) initWheels();

  // Initialize the ach channels to the waist daemon and the waist state
  // channel. The daemon moves the motors in unison with current control and
  // pciod updates their state on the state channel.
  if (mode & MODE_WAIST) initWaist();

  // initial coordinate values for updating position coordinates that are to be
  // found by integration
  position_on_ground_plane.x = 0.0;
  position_on_ground_plane.y = 0.0;

  // Updates the robot kinematics
  double dt = 0.0;
  updateKinematics(dt);

  // Determine the type of grippers from input mode
  //	FT::GripperType ft_grippers;
  //	if(mode & MODE_GRIPPERS) ft_grippers = FT::GRIPPER_TYPE_ROBOTIQ;
  //	if(mode & MODE_GRIPPERS_SCH) ft_grippers = FT::GRIPPER_TYPE_SCHUNK;

  // After initializing the rest of the robot (need kinematics), we can
  // initialize f/t sensors.
  //	if(mode & MODE_LARM) fts[LEFT] = new FT(ft_grippers, daemon_cx, robot,
  // LEFT); 	if(mode & MODE_RARM) fts[RIGHT] = new FT(ft_grippers, daemon_cx,
  // robot, RIGHT);
}

/* ********************************************************************************************
 */
Hardware::~Hardware() {
  // Close imu channel and the filter
  somatic_d_channel_close(daemon_cx, imu_chan);
  delete imu_chan;
  if (kfImu) filter_kalman_destroy(kfImu);

  // Destroy the ft sensors
  //	if(fts[LEFT] != NULL) delete fts[LEFT];
  //	if(fts[RIGHT] != NULL) delete fts[RIGHT];

  // Send zero velocity to amc and clean it up
  double zeros2[2] = {0.0, 0.0},
         zeros7[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  if (amc != NULL) {
    somatic_motor_cmd(daemon_cx, amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,
                      zeros2, 2, NULL);
    somatic_motor_destroy(daemon_cx, amc);
    delete amc;
  }

  // Send halt messages directly to the left and right arms
  // TODO: should we be halting the arms?
  if (arms[RIGHT] != NULL) {
    somatic_motor_halt(daemon_cx, arms[RIGHT]);
    somatic_motor_destroy(daemon_cx, arms[RIGHT]);
    delete arms[RIGHT];
  }
  if (arms[LEFT] != NULL) {
    somatic_motor_halt(daemon_cx, arms[LEFT]);
    somatic_motor_destroy(daemon_cx, arms[LEFT]);
    delete arms[LEFT];
  }

  // Destroy the gripper motors
  if (grippers[LEFT] != NULL) {
    somatic_motor_halt(daemon_cx, grippers[LEFT]);
    somatic_motor_destroy(daemon_cx, grippers[LEFT]);
    delete grippers[LEFT];
  }
  if (grippers[RIGHT] != NULL) {
    somatic_motor_halt(daemon_cx, grippers[RIGHT]);
    somatic_motor_destroy(daemon_cx, grippers[RIGHT]);
    delete grippers[RIGHT];
  }

  // Stop and destroy	waist motors
  if (waist != NULL) {
    // Create a waist daemon message with the stop mode
    Somatic__WaistCmd* waistDaemonCmd = somatic_waist_cmd_alloc();
    somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);

    // Send the message
    int r = SOMATIC_PACK_SEND(waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
    if (ACH_OK != r)
      fprintf(stderr, "Couldn't send stop message to waist: %s\n",
              ach_result_to_string(static_cast<ach_status_t>(r)));

    // Clean up the memory
    somatic_motor_destroy(daemon_cx, waist);
    delete waist;
  }
}

/* ********************************************************************************************
 */
void Hardware::initWaist() {
  // Initialize the channel to the waist daemon
  waistCmdChan = new ach_channel_t();
  somatic_d_channel_open(daemon_cx, waistCmdChan, "waistd-cmd", NULL);

  // Initialize the somatic motor interface to listen to the state
  waist = new somatic_motor_t();
  somatic_motor_init(daemon_cx, waist, 2, NULL, "waist-state");
  usleep(1e5);

  // Set the min/max values for the pos/vel fields' valid and limit values
  VectorXd lim2 = VectorXd::Ones(2) * 1024.1;
  for (int i = 0; i < 2; i++) {
    waist->pos_valid_min[i] = -lim2[i];
    waist->pos_limit_min[i] = -lim2[i];
    waist->pos_valid_max[i] = lim2[i];
    waist->pos_limit_max[i] = lim2[i];

    waist->vel_valid_min[i] = -lim2[i];
    waist->vel_limit_min[i] = -lim2[i];
    waist->vel_valid_max[i] = lim2[i];
    waist->vel_limit_max[i] = lim2[i];
  }

  // Get an update
  somatic_motor_update(daemon_cx, waist);
  usleep(1e5);
}

/* ********************************************************************************************
 */
void Hardware::updateSensors(double dt) {
  // Update the lower body motors to get the current values
  if (mode & MODE_AMC) somatic_motor_update(daemon_cx, amc);
  if (mode & MODE_TORSO) somatic_motor_update(daemon_cx, torso);
  if (mode & MODE_WAIST) somatic_motor_update(daemon_cx, waist);

	// Update the imu
	getImu(imu_chan, imu, imuSpeed, dt, kfImu, &rawImu, &rawImuSpeed);

  // Update the arms
  if (mode & MODE_LARM) somatic_motor_update(daemon_cx, arms[LEFT]);
  if (mode & MODE_RARM) somatic_motor_update(daemon_cx, arms[RIGHT]);

  // Update the kinematics
  updateKinematics(dt);

  // Update the f/t readings
  //	if(mode & MODE_LARM) fts[LEFT]->updateExternal();
  //	if(mode & MODE_RARM) fts[RIGHT]->updateExternal();
}
/* ********************************************************************************************
 */
Eigen::Vector3d GetBodyCom(dart::dynamics::SkeletonPtr robot) {
  dart::dynamics::BodyNodePtr lwheel = robot->getBodyNode("LWheel");
  dart::dynamics::BodyNodePtr rwheel = robot->getBodyNode("RWheel");
  double wheel_mass = lwheel->getMass();
  double full_mass = robot->getMass();
  return (full_mass * robot->getCOM() - wheel_mass * lwheel->getCOM() -
          wheel_mass * rwheel->getCOM()) /
         (full_mass - 2 * wheel_mass);
}

/* ********************************************************************************************
 */
void Hardware::updateKinematics(double dt) {
  double R = 0.25;  // wheel radius
  double L = 0.68;  // distance between wheels

  // -- Base frame
  // orientation
  double heading = R / L * (amc->pos[1] - amc->pos[0]);
  double q_base = imu;
  auto base_tf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  base_tf.prerotate(Eigen::AngleAxisd(-q_base, Eigen::Vector3d::UnitX()))
      .prerotate(
          Eigen::AngleAxisd(-M_PI / 2 + heading, Eigen::Vector3d::UnitY()))
      .prerotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  auto aa = Eigen::AngleAxisd(base_tf.rotation());

  // positions
  double heading_speed = R * ((amc->vel[0] + amc->vel[1]) / 2.0 + imuSpeed);
  position_on_ground_plane.x += heading_speed * cos(heading) * dt;
  position_on_ground_plane.y += heading_speed * sin(heading) * dt;
  Eigen::Vector3d xyz;
  xyz << position_on_ground_plane.x, position_on_ground_plane.y, R;

  // set position coordinates in dart skeleton
  Eigen::Matrix<double, 6, 1> q_base_dart;
  q_base_dart << aa.angle() * aa.axis(), xyz;
  robot->setPositions({0, 1, 2, 3, 4, 5}, q_base_dart);

  // angular speed
  double dq_base = imuSpeed;
  double dq_yaw = R / L * (amc->vel[1] - amc->vel[0]);
  Eigen::Vector3d base_angular_velocity;  // in local frame
  base_angular_velocity << -dq_base, dq_yaw * cos(q_base),
      dq_yaw * sin(q_base);  // see transform Jacobian derivation in Munzir's
                             // thesis for this

  // linear speed
  Eigen::Vector3d base_linear_velocity;  // in local frame
  base_linear_velocity << 0.0, heading_speed * sin(q_base),
      -heading_speed * cos(q_base);  // see transform Jacobian derivation in
                                     // Munzir's thesis for this

  // set speed coordinates in dart skeleton
  Eigen::Matrix<double, 6, 1> dq_base_dart;
  q_base_dart << base_angular_velocity, base_linear_velocity;
  robot->setVelocities({0, 1, 2, 3, 4, 5}, dq_base_dart);

  // -- Left wheel
  double q_lwheel = amc->pos[0];
  robot->getJoint("JLWheel")->setPosition(0, q_lwheel);
  double dq_lwheel = amc->vel[0];
  robot->getJoint("JLWheel")->setVelocity(0, dq_lwheel);

  // -- Right wheel
  double q_rwheel = amc->pos[1];
  robot->getJoint("JRWheel")->setPosition(0, q_rwheel);
  double dq_rwheel = amc->vel[1];
  robot->getJoint("JRWheel")->setVelocity(0, dq_rwheel);

  // -- Waist
  double q_waist = (waist->pos[0] - waist->pos[1]) / 2.0;
  robot->getJoint("JWaist")->setPosition(0, q_waist);
  double dq_waist = (waist->vel[0] - waist->vel[1]) / 2.0;
  robot->getJoint("JWaist")->setVelocity(0, dq_waist);

  // -- Torso
  double q_torso = torso->pos[0];
  robot->getJoint("JTorso")->setPosition(0, q_torso);
  double dq_torso = torso->vel[0];
  robot->getJoint("JTorso")->setVelocity(0, dq_torso);

  // -- Arms
  Eigen::Matrix<double, 7, 1> q_left_arm, dq_left_arm, q_right_arm,
      dq_right_arm;
  q_left_arm = eig7(arms[LEFT]->pos);
  dq_left_arm = eig7(arms[LEFT]->vel);
  q_right_arm = eig7(arms[RIGHT]->pos);
  dq_right_arm = eig7(arms[RIGHT]->vel);
  std::vector<std::string> left_arm_joint_names = {"LJ1", "LJ2", "LJ3", "LJ4",
                                                   "LJ5", "LJ6", "LJFT"};
  std::vector<std::string> right_arm_joint_names = {"RJ1", "RJ2", "RJ3", "RJ4",
                                                    "RJ5", "RJ6", "RJFT"};
  for (int i = 0; i < 7; i++) {
    robot->getJoint(left_arm_joint_names[i])->setPosition(0, q_left_arm(i));
    robot->getJoint(left_arm_joint_names[i])->setVelocity(0, dq_left_arm(i));
    robot->getJoint(right_arm_joint_names[i])->setPosition(0, q_right_arm(i));
    robot->getJoint(right_arm_joint_names[i])->setVelocity(0, dq_right_arm(i));
  }
}

/* ********************************************************************************************
 */
void Hardware::initImu(bool filter_imu) {
  // Initialize the ach channel
  imu_chan = new ach_channel_t();
  somatic_d_channel_open(daemon_cx, imu_chan, "imu-data", NULL);

  // Average the first second's worth of readings
  double time_ft_av_start = aa_tm_timespec2sec(aa_tm_now());
  int num_data = 0;
  imu = imuSpeed = 0.0;
  while ((num_data < 100) ||
         (aa_tm_timespec2sec(aa_tm_now()) - time_ft_av_start < 1.0)) {
    double tempImu, tempImuSpeed;
    getImu(imu_chan, tempImu, tempImuSpeed, 0.0, NULL);
    imu += tempImu, imuSpeed += tempImuSpeed;
    num_data++;
  }
  imu /= (double)num_data, imuSpeed /= (double)num_data;

  // Create the kalman filter
  if (filter_imu) {
    kfImu = new filter_kalman_t;
    filter_kalman_init(kfImu, 2, 0, 2);
    kfImu->C[0] = kfImu->C[3] = 1.0;
    kfImu->Q[0] = kfImu->Q[3] = 1e-3;
    kfImu->x[0] = imu, kfImu->x[1] = imuSpeed;
  } else {
    kfImu = NULL;
  }
}

/* ********************************************************************************************
 */
void Hardware::initWheels() {
  // Initialize the motor group (do we need to set any limits?)
  amc = new somatic_motor_t();
  somatic_motor_init(daemon_cx, amc, 2, "amc-cmd", "amc-state");

  // Set the min/max values for the pos/vel fields' valid and limit values
  for (int i = 0; i < 2; i++) {
    amc->pos_valid_min[i] = -1024.1;
    amc->pos_valid_max[i] = 1024.1;
    amc->pos_limit_min[i] = -1024.1;
    amc->pos_limit_max[i] = 1024.1;

    amc->vel_valid_min[i] = -1024.1;
    amc->vel_valid_max[i] = 1024.1;
    amc->vel_limit_min[i] = -1024.1;
    amc->vel_limit_max[i] = 1024.1;
  }

  // Update and reset them
  somatic_motor_update(daemon_cx, amc);
  usleep(1e5);

  // Set the offset values to amc motor group so initial wheel pos readings are
  // zero
  double pos_offset[2] = {-amc->pos[0] - imu, -amc->pos[1] - imu};
  aa_fcpy(amc->pos_offset, pos_offset, 2);
  usleep(1e5);
}

/* ********************************************************************************************
 */
void Hardware::initMotorGroup(somatic_d_t* daemon_cx, somatic_motor_t*& motors,
                              const char* cmd_name, const char* state_name,
                              VectorXd minPos, VectorXd maxPos, VectorXd minVel,
                              VectorXd maxVel) {
  // Initialize the somatic motor struct with the channel names and the number
  // of modules
  motors = new somatic_motor_t();
  size_t numModules = minPos.size();
  assert(minPos.size() == maxPos.size() && minPos.size() == minVel.size() &&
         minPos.size() == maxVel.size());
  somatic_motor_init(daemon_cx, motors, numModules, cmd_name, state_name);

  // Set the min/max values for the pos/vel fields' valid and limit values
  for (int i = 0; i < numModules; i++) {
    motors->pos_valid_min[i] = minPos[i];
    motors->pos_valid_max[i] = maxPos[i];
    motors->pos_limit_min[i] = minPos[i];
    motors->pos_limit_max[i] = maxPos[i];

    motors->vel_valid_min[i] = minVel[i];
    motors->vel_valid_max[i] = maxVel[i];
    motors->vel_limit_min[i] = minVel[i];
    motors->vel_limit_max[i] = maxVel[i];
  }

  // Update and reset them
  //	somatic_motor_reset(daemon_cx, motors);
  //	usleep(1e5);
  somatic_motor_update(daemon_cx, motors);
  usleep(1e5);
}

/* ********************************************************************************************
 */
/*void Hardware::printState () {
        VectorXd s = robot->getPose();
        printf("imu: %.3lf, waist: %.3lf, torso: %.3lf\n", s(imuWaist_ids[0]),
s(imuWaist_ids[1]), s(9)); printf("left arm: ("); for(size_t i = 0; i < 7; i++)
printf(" %.3lf,", s(left_arm_ids[i])); printf("\b)\nright arm: ("); for(size_t i
= 0; i < 7; i++) printf(" %.3lf,", s(right_arm_ids[i])); printf("\b)\n");
}*/

/* ********************************************************************************************
 */
/*void Hardware::printStateCurses(int row, int col) {
        VectorXd s = robot->getPose();
        mvprintw(row, col, "Robot hardware state:");
        mvprintw(row+1, col+1, "imu: %.3lf", s(imuWaist_ids[0]));
        mvprintw(row+1, col+16, "waist: %.3lf", s(imuWaist_ids[1]));
        mvprintw(row+1, col+31, "torso: %.3lf", s(9));

        // display amc currents
        char bar_buffer[1024];
        int bar_len = 80;
        double amc_max = 40;
        for(int mot = 0; mot < amc->n; mot++) {
                int i = 0;
                for(; i < bar_len * amc->cur[mot] / amc_max; i++)
bar_buffer[bar_len + i] = '|'; for(; i < bar_len; i++) bar_buffer[bar_len + i] =
' '; i = 0; for(; i < bar_len * -amc->cur[mot] / amc_max; i++)
bar_buffer[bar_len - i] = '|'; for(; i < bar_len; i++) bar_buffer[bar_len - i] =
' '; bar_buffer[2*bar_len-1] = '\0'; bar_buffer[0] = '[';
                bar_buffer[2*bar_len-2] = ']';
                mvprintw(row+3+mot, col, bar_buffer);
        }

        // print out the left arm, with pretty colors for joint limits
        mvprintw(row+6, col+2, "left arm:");
        kinematics::Dof* dof;
        double dist;
        for(int i = 0; i < 7; i++) {
                dof = robot->getDof(left_arm_ids[i]);
                dist = std::min(std::abs(s[left_arm_ids[i]] - dof->getMin()),
                                                                                std::abs(s[left_arm_ids[i]] - dof->getMax()));
                if (dist < .25) attron(COLOR_PAIR(COLOR_RED_BACKGROUND));
                mvprintw(row+6, col+14+(i*12), "%.8lf", s(left_arm_ids[i]));
                if (dist < .25) attroff(COLOR_PAIR(COLOR_RED_BACKGROUND));
        }

        // the same for the right arm. TODO: unify these
        mvprintw(row+7, col+2, "right arm:");
        for(int i = 0; i < 7; i++) {
                dof = robot->getDof(right_arm_ids[i]);
                dist = std::min(std::abs(s[right_arm_ids[i]] - dof->getMin()),
                                                                                std::abs(s[right_arm_ids[i]] - dof->getMax()));
                if (dist < .25) attron(COLOR_PAIR(COLOR_RED_BACKGROUND));
                mvprintw(row+7, col+14+(i*12), "%.8lf", s(right_arm_ids[i]));
                if (dist < .25) attroff(COLOR_PAIR(COLOR_RED_BACKGROUND));
        }
}*/

};  // namespace Krang
