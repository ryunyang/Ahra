#include "pioneer_plugin.hh"
#include "Walkingpattern_generator.hh"
#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323
namespace gazebo
{
void pioneer::Load(ModelPtr _model, sdf::ElementPtr)
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "pioneer");
    ROS_INFO("Seonghyeon");

    sub = n.subscribe("turn_angle", 1, &pioneer::TurnCallback, this);
    sub_motion_selector = n.subscribe("motion_selector", 1, &pioneer::SelectMotion, this);
    this->model = _model;
    jc0 = new physics::JointController(model);
    GetLinks();
    GetJoints();
    GetSensor();
    MotionMaker();
    IdleMotion();
    InitROSPubSetting();
    // this->last_update_time = this->model->GetWorld()->SimTime();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&pioneer::OnUpdate, this, _1));
    this->RL_Sensor->SetActive(true);
    this->LL_Sensor->SetActive(true);
    this->ImuSensor->SetActive(true);
    time = 0;
    indext = 0;
    angle = 0;
    // all_theta_data = fopen("/home/jaemin/matlab_codes/all_theta_data.dat", "w");
    // // Imu_pos = fopen("/home/jaemin/matlab_codes/Imu_pos.dat", "w");
    // ref_theta_data = fopen("/home/jaemin/matlab_codes/ref_theta_data.dat", "w");
  }

void pioneer::InitROSPubSetting()
{
    pub_joint_state = n.advertise<sensor_msgs::JointState>("pioneer/joint_states", 100);
}

void pioneer::ROSMsgPublish()
{
 sensor_msgs::JointState joint_state_msg;

 joint_state_msg.header.stamp = ros::Time::now();
 for (uint8_t i = 0; i < 12; i++)
 {
   joint_state_msg.name.push_back((std::string)joint_names.at(i));
   joint_state_msg.position.push_back((float)(sensor_th[i]));
 }
 pub_joint_state.publish(joint_state_msg);
}

void pioneer::GetLinks(){
    this->base_link  = this->model->GetLink("base_link");
    this->RL_link1 = this->model->GetLink("RL_link1");
    this->RL_link2 = this->model->GetLink("RL_link2");
    this->RL_link3 = this->model->GetLink("RL_link3");
    this->RL_link4 = this->model->GetLink("RL_link4");
    this->RL_link5 = this->model->GetLink("RL_link5");
    this->RL_link6 = this->model->GetLink("RL_link6");
    this->LL_link1 = this->model->GetLink("LL_link1");
    this->LL_link2 = this->model->GetLink("LL_link2");
    this->LL_link3 = this->model->GetLink("LL_link3");
    this->LL_link4 = this->model->GetLink("LL_link4");
    this->LL_link5 = this->model->GetLink("LL_link5");
    this->LL_link6 = this->model->GetLink("LL_link6");
  }

void pioneer::GetJoints()
  {
    this->RL_j1 = this->model->GetJoint("RL_j1");
    this->RL_j2 = this->model->GetJoint("RL_j2");
    this->RL_j3 = this->model->GetJoint("RL_j3");
    this->RL_j4 = this->model->GetJoint("RL_j4");
    this->RL_j5 = this->model->GetJoint("RL_j5");
    this->RL_j6 = this->model->GetJoint("RL_j6");

    this->LL_j1 = this->model->GetJoint("LL_j1");
    this->LL_j2 = this->model->GetJoint("LL_j2");
    this->LL_j3 = this->model->GetJoint("LL_j3");
    this->LL_j4 = this->model->GetJoint("LL_j4");
    this->LL_j5 = this->model->GetJoint("LL_j5");
    this->LL_j6 = this->model->GetJoint("LL_j6");

    this->RA_j1 = this->model->GetJoint("RA_j1");
    this->RA_j2 = this->model->GetJoint("RA_j2");
    this->RA_j3 = this->model->GetJoint("RA_j3");
    this->RA_j4 = this->model->GetJoint("RA_j4");
    this->LA_j1 = this->model->GetJoint("LA_j1");
    this->LA_j2 = this->model->GetJoint("LA_j2");
    this->LA_j3 = this->model->GetJoint("LA_j3");
    this->LA_j4 = this->model->GetJoint("LA_j4");
    this->bodyj = this->model->GetJoint("bodyj");
    this->Neck_j1 = this->model->GetJoint("Neck_j1");
    this->Neck_j2 = this->model->GetJoint("Neck_j2");


  }

void pioneer::OnUpdate(const common::UpdateInfo &)
  { 
    current_time = this->model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    last_update_time = current_time;
    GetSensorValues();
    GetJointPosition();
    ROSMsgPublish();
    PostureGeneration();
    PIDcontroller();
    SetTorque();
    // MakeMatlabFile();

  }

void pioneer::SelectMotion(const std_msgs::Float32Ptr &msg){
    mode = msg ->data;
    // ROS_INFO("mode(%f)",mode);
    if (indext == 0){
    if (mode == 0 ){
    ref_LL_th = ref_LL_th0;
    ref_RL_th = ref_RL_th0;
    }
    else if (mode == 1){
    ref_LL_th = ref_LL_th1;
    ref_RL_th = ref_RL_th1;
    }
    else if (mode == 2)
    {
    ref_LL_th = ref_LL_th2;
    ref_RL_th = ref_RL_th2;
    }
    else if (mode == 3)
    {
    ref_LL_th = ref_LL_th3;
    ref_RL_th = ref_RL_th3;
    }
    else if (mode == 4)
    {
    ref_LL_th = ref_LL_th4;
    ref_RL_th = ref_RL_th4;
    }
    else if (mode == 5)
    {
    ref_LL_th = ref_LL_th5;
    ref_RL_th = ref_RL_th5;
    }
    else if (mode == 6)
    {
    ref_LL_th = ref_LL_th6;
    ref_RL_th = ref_RL_th6;
    }
    else if (mode == 7)
    {
    ref_LL_th = ref_LL_th7;
    ref_RL_th = ref_RL_th7;
    }
    else if (mode == 8)
    {
    ref_LL_th = ref_LL_th8;
    ref_RL_th = ref_RL_th8;
    }
    else
    {
    ref_LL_th = ref_LL_th0;
    ref_RL_th = ref_RL_th0;
    }
    }
}

void pioneer::SetJointPosition()
{
    RL_j1->SetPosition(0, RL_th(0));
    RL_j2->SetPosition(0, RL_th(1));
    RL_j3->SetPosition(0, RL_th(2));
    RL_j4->SetPosition(0, RL_th(3));
    RL_j5->SetPosition(0, RL_th(4));
    RL_j6->SetPosition(0, RL_th(5));
    LL_j1->SetPosition(0, LL_th(0));
    LL_j2->SetPosition(0, LL_th(1));
    LL_j3->SetPosition(0, LL_th(2));
    LL_j4->SetPosition(0, LL_th(3));
    LL_j5->SetPosition(0, LL_th(4));
    LL_j6->SetPosition(0, LL_th(5));
    bodyj->SetPosition(0, 0);
    RA_j1->SetPosition(0, 0);
    RA_j2->SetPosition(0, 90 * deg2rad);
    RA_j3->SetPosition(0, 90 * deg2rad);
    RA_j4->SetPosition(0, 0);
    LA_j1->SetPosition(0, 0);
    LA_j2->SetPosition(0, -90 * deg2rad);
    LA_j3->SetPosition(0, -90 * deg2rad);
    LA_j4->SetPosition(0, 0);
    Neck_j1->SetPosition(0, 0);
    Neck_j2->SetPosition(0, 0);
}

void pioneer::GetJointPosition()
{
    sensor_th[0] = this->RL_j1->Position(0);
    sensor_th[1] = this->RL_j2->Position(0);
    sensor_th[2] = this->RL_j3->Position(0);
    sensor_th[3] = this->RL_j4->Position(0);
    sensor_th[4] = this->RL_j5->Position(0);
    sensor_th[5] = this->RL_j6->Position(0);
    sensor_th[6] = this->LL_j1->Position(0);
    sensor_th[7] = this->LL_j2->Position(0);
    sensor_th[8] = this->LL_j3->Position(0);
    sensor_th[9] = this->LL_j4->Position(0);
    sensor_th[10] = this->LL_j5->Position(0);
    sensor_th[11] = this->LL_j6->Position(0);
    sensor_th[12] = this->bodyj->Position(0);
    sensor_th[13] = this->RA_j1->Position(0);
    sensor_th[14] = this->RA_j2->Position(0);
    sensor_th[15] = this->RA_j3->Position(0);
    sensor_th[16] = this->RA_j4->Position(0);
    sensor_th[17] = this->LA_j1->Position(0);
    sensor_th[18] = this->LA_j2->Position(0);
    sensor_th[19] = this->LA_j3->Position(0);
    sensor_th[20] = this->LA_j4->Position(0);
    sensor_th[21] = this->Neck_j1->Position(0);
    sensor_th[22] = this->Neck_j2->Position(0);
}

void pioneer::TurnCallback(const std_msgs::Float32Ptr &msg)
{
    angle = msg->data;
    // ROS_INFO("angle callback(%f)",angle);
    if (angle < 0)
    {
    TurningTrajectory_LL(angle);
    }
    else if (angle > 0)
    {
    TurningTrajectory_RL(angle);
    }
};

void pioneer::PostureGeneration()
{
    time += 1;
    if (time >= 2)
    {
    RL_th(0) = ref_RL_th(indext, 0);
    RL_th(1) = ref_RL_th(indext, 1);
    RL_th(2) = -ref_RL_th(indext, 2);
    RL_th(3) = -ref_RL_th(indext, 3);
    RL_th(4) = -ref_RL_th(indext, 4);
    RL_th(5) = ref_RL_th(indext, 5);
    LL_th(0) = ref_LL_th(indext, 0);
    LL_th(1) = ref_LL_th(indext, 1);
    LL_th(2) = ref_LL_th(indext, 2);
    LL_th(3) = ref_LL_th(indext, 3);
    LL_th(4) = ref_LL_th(indext, 4);
    LL_th(5) = ref_LL_th(indext, 5);
    if (indext == simt * 1.75 && RL_contacts.contact_size() == 0)
    {
    indext = indext;
    }
    else if (indext == simt * 2.75 && RL_contacts.contact_size() == 0)
    {
    indext = indext;
    }
    else if (indext == simt * 3.75 && RL_contacts.contact_size() == 0)
    {
    indext = indext;
    }
    else if (indext == simt * 1.25 && LL_contacts.contact_size() == 0)
    {
    indext = indext;
    }
    else if (indext == simt * 2.25 && LL_contacts.contact_size() == 0)
    {
    indext = indext;
    }
    else if (indext == simt * 3.25 && LL_contacts.contact_size() == 0)
    {
    indext = indext;
    }
    else
    {
    indext += 1;
    }
    time = 0;
    }
    if (indext >= ref_RL_th.rows())
    {    
    ref_LL_th = ref_LL_th0;
    ref_RL_th = ref_RL_th0;
    if (RL_contacts.contact_size() == 1 && LL_contacts.contact_size() == 1)
    {
    indext = 0;
    }
    else
    indext = indext - 1;
    }
}

 void pioneer::IdleMotion()
  { 
    RL_j1->SetPosition(0, ref_RL_th0(0,0));
    RL_j2->SetPosition(0, ref_RL_th0(0,1));
    RL_j3->SetPosition(0, -ref_RL_th0(0,2));
    RL_j4->SetPosition(0, -ref_RL_th0(0,3));
    RL_j5->SetPosition(0, -ref_RL_th0(0,4));
    RL_j6->SetPosition(0, ref_RL_th0(0,5));
    LL_j1->SetPosition(0, ref_LL_th0(0,0));
    LL_j2->SetPosition(0, ref_LL_th0(0,1));
    LL_j3->SetPosition(0, ref_LL_th0(0,2));
    LL_j4->SetPosition(0, ref_LL_th0(0,3));
    LL_j5->SetPosition(0, ref_LL_th0(0,4));
    LL_j6->SetPosition(0, ref_LL_th0(0,5));
    bodyj->SetPosition(0,0);
    RA_j1->SetPosition(0,0);
    RA_j2->SetPosition(0,90*deg2rad);
    RA_j3->SetPosition(0,-90*deg2rad);
    RA_j4->SetPosition(0,0);
    LA_j1->SetPosition(0,0);
    LA_j2->SetPosition(0,-90*deg2rad);
    LA_j3->SetPosition(0,90*deg2rad);
    LA_j4->SetPosition(0,0);
    Neck_j1->SetPosition(0,0);
    Neck_j2->SetPosition(0,0);

 } 

void pioneer::PIDcontroller(){
  // p,d게인 for 33
// double kp[23] = {100,450,450,400,500,700/*Rleg*/,100,450,450,400,500,700,/*Lleg*/100,100,100,100,100,100,100,100,100,100,100};
// double kd[23] = {0.01,0.01,0.01,0.01,0.01,0.01,/*Rleg*/0.01,0.01,0.01,0.01,0.01,0.01,/*Lleg*/0,0,0,0,0,0,0,0,0,0,0};
//pdgain for 3
double kp[23] = {600,600, 600, 600, 600,600/*Rleg*/,600,600, 600, 600, 600,600,/*Lleg*/100,100,100,100,100,100,100,100,100,100,100};
double kd[23] = {0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,/*Rleg*/0.0001,0.0001,0.00001,0.0001,0.0001,0.0001,/*Lleg*/0,0,0,0,0,0,0,0,0,0,0};

//arm degree
ref_RA_th << 0, 90*deg2rad,-90*deg2rad ,0;
ref_LA_th << 0, -90*deg2rad,90*deg2rad ,0;

//error, errordot
for (int i = 0; i<6;i++){
  error[i] = RL_th(i)-sensor_th[i];
  error[i+6] = LL_th(i)-sensor_th[i+6];
}
error[12] = 0 - sensor_th[12];
for (int i = 0; i<4; i++){
  error[i+13] = ref_RA_th[i]- sensor_th[i+13];
  error[i+17] = ref_LA_th[i]- sensor_th[i+17];
}
error[21] = 0-sensor_th[21];
error[22] = 0-sensor_th[22];

for (int i = 0; i<23;i++){
  error_dot[i] = (sensor_th[i] - prev_position[i]) / dt;
  torque[i] = kp[i]*error[i] + kd[i]*error_dot[i];
}
  prev_position = sensor_th;
}

void pioneer::SetTorque(){
  RL_j1->SetForce(0, torque(0));
  RL_j2->SetForce(0, torque(1));
  RL_j3->SetForce(0, torque(2));
  RL_j4->SetForce(0, torque(3));
  RL_j5->SetForce(0, torque(4));
  RL_j6->SetForce(0, torque(5));
  LL_j1->SetForce(0, torque(6));
  LL_j2->SetForce(0, torque(7));
  LL_j3->SetForce(0, torque(8));
  LL_j4->SetForce(0, torque(9));
  LL_j5->SetForce(0, torque(10));
  LL_j6->SetForce(0, torque(11));
  bodyj->SetForce(0, torque(12));
  RA_j1->SetForce(0, torque(13));
  RA_j2->SetForce(0, torque(14));
  RA_j3->SetForce(0, torque(15));
  RA_j4->SetForce(0, torque(16));
  LA_j1->SetForce(0, torque(17));
  LA_j2->SetForce(0, torque(18));
  LA_j3->SetForce(0, torque(19));
  LA_j4->SetForce(0, torque(20));
  Neck_j1->SetForce(0, torque(21));
  Neck_j2->SetForce(0, torque(22));

}

void pioneer::MotionMaker(){

    Motions motion;
    motion.Motion0();
    ref_LL_th0 = motion.Return_Motion0_LL();
    ref_RL_th0 = motion.Return_Motion0_RL();

    motion.Motion1();
    ref_LL_th1 = motion.Return_Motion1_LL();
    ref_RL_th1 = motion.Return_Motion1_RL();

    motion.Motion2();
    ref_LL_th2 = motion.Return_Motion2_LL();
    ref_RL_th2 = motion.Return_Motion2_RL();

    motion.Motion3();
    ref_LL_th3 = motion.Return_Motion3_LL();
    ref_RL_th3 = motion.Return_Motion3_RL();

    motion.Motion4();
    ref_LL_th4 = motion.Return_Motion4_LL();
    ref_RL_th4 = motion.Return_Motion4_RL();

    motion.Motion5();
    ref_LL_th5 = motion.Return_Motion5_LL();
    ref_RL_th5 = motion.Return_Motion5_RL();

    motion.Motion6();
    ref_LL_th6 = motion.Return_Motion6_LL();
    ref_RL_th6 = motion.Return_Motion6_RL();

    motion.Motion7();
    ref_LL_th7 = motion.Return_Motion7_LL();
    ref_RL_th7 = motion.Return_Motion7_RL();

    ref_LL_th = ref_LL_th0;
    ref_RL_th = ref_RL_th0;
}

void pioneer::TurningTrajectory_LL(double angle)
{
    Foot equationsolver;
    this->TurningTrajectory_coeff = equationsolver.Equation_solver(0, walktime * 0.3, 0, angle);
    this->BackTrajectory_coeff = equationsolver.Equation_solver(0, walktime * 0.3, angle, 0);

    if (indext < 1.6 * walktime * 500)
    {
  for (int i = 0; i < sim_n; i++)
  {
      double time = i*del_t;
      if (1.6 * walktime < time && time < 1.9 * walktime) 
      {
        ref_RL_th(i, 0) = Turn(time - 1.6 * walktime);
      }
      else if (1.9 * walktime < time && time < 2.1 * walktime)
      {
        ref_RL_th(i, 0) = angle;
      }
      else if (2.1 * walktime < time && time < 2.4 * walktime)
      {
        ref_RL_th(i, 0) = Back(time - 2.1 * walktime);
      }

  }
    }
    else if (indext < 2.6 * walktime * 500)
    {
  for (int j = 0; j < sim_n; j++)
  {
      double time = j*del_t;
      if (2.6 * walktime < time && time < 2.9 * walktime) 
      {
        ref_RL_th(j, 0) = Turn(time - 2.6 * walktime);
      }
      else if (2.9 * walktime < time && time < 3.1 * walktime)
      {
        ref_RL_th(j, 0) = angle;
      }
      else if (3.1 * walktime < time && time < 3.4 * walktime)
      {
        ref_RL_th(j, 0) = Back(time - 3.1 * walktime);
      }
  }
    }
    else if (indext < 3.6 * walktime * 500)
    {
  for (int k = 0; k < sim_n; k++)
  {
      double time = k*del_t;
      if (3.6 * walktime < time && time < 3.9 * walktime) // swing
      {
        ref_RL_th(k, 0) = Turn(time - 3.6 * walktime);
      }
      else if (3.9 * walktime < time && time < 4.1 * walktime)
      {
        ref_RL_th(k, 0) = angle;
      }
      else if (4.1 * walktime < time && time < 4.4 * walktime)
      {
        ref_RL_th(k, 0) = Back(time - 4.1 * walktime);
      }
  }
    }
}

void pioneer::TurningTrajectory_RL(double angle){
    Foot equationsolver;
    this->TurningTrajectory_coeff = equationsolver.Equation_solver(0, walktime * 0.3, 0, angle);
    this->BackTrajectory_coeff = equationsolver.Equation_solver(0, walktime * 0.3, angle, 0);

    if (indext < 1.1 * walktime * 500)
    {
  for (int i = 0; i < sim_n; i++)
  {
      double time = i*del_t;
      if (1.1 * walktime < time && time < 1.4 * walktime) 
      {
        ref_RL_th(i, 0) = Turn(time - 1.1 * walktime);
      }
      else if (1.4 * walktime < time && time < 1.6 * walktime)
      {
        ref_RL_th(i, 0) = angle;
      }
      else if (1.6 * walktime < time && time < 1.9 * walktime)
      {
        ref_RL_th(i, 0) = Back(time - 1.6 * walktime);
      }

  }
    }
    else if (indext < 2.1 * walktime * 500)
    {
  for (int j = 0; j < sim_n; j++)
  {
      double time = j*del_t;
      if (2.1 * walktime < time && time < 2.4 * walktime) 
      {
        ref_RL_th(j, 0) = Turn(time - 2.1 * walktime);
      }
      else if (2.4 * walktime < time && time < 2.6 * walktime)
      {
        ref_RL_th(j, 0) = angle;
      }
      else if (2.6 * walktime < time && time < 2.9 * walktime)
      {
        ref_RL_th(j, 0) = Back(time - 2.6 * walktime);
      }
  }
    }
    else if (indext < 3.1 * walktime * 500)
    {
  for (int k = 0; k < sim_n; k++)
  {
      double time = k*del_t;
      if (3.1 * walktime < time && time < 3.4 * walktime) // swing
      {
        ref_RL_th(k, 0) = Turn(time - 3.1 * walktime);
      }
      else if (3.4 * walktime < time && time < 3.6 * walktime)
      {
        ref_RL_th(k, 0) = angle;
      }
      else if (3.6 * walktime < time && time < 3.9 * walktime)
      {
        ref_RL_th(k, 0) = Back(time - 3.6 * walktime);
      }
  }
    }

}

void pioneer::GetSensor()
{

    this->RL_Sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(gazebo::sensors::SensorManager::Instance()->GetSensor("RL_contact_sensor"));
    if (!this->RL_Sensor)
    {
  ROS_INFO("ContactPlugin requires a RLContactSensor.\n");
  gzerr << "ContactPlugin requires a RLContactSensor.\n";
  return;
    }
    this->LL_Sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(gazebo::sensors::SensorManager::Instance()->GetSensor("LL_contact_sensor"));
    if (!this->LL_Sensor)
    {
  ROS_INFO("ContactPlugin requires a LLContactSensor.\n");
  gzerr << "ContactPlugin requires a LLContactSensor.\n";
  return;
    }

    this->ImuSensor = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(gazebo::sensors::SensorManager::Instance()->GetSensor("imu_sensor"));
    if (!this->ImuSensor)
    {
  std::cerr << "IMU sensor not found!" << std::endl;
  return;
    }
}

void pioneer::GetSensorValues(){
RL_contacts = this->RL_Sensor->Contacts();
LL_contacts = this->LL_Sensor->Contacts();
angularVelocity = ImuSensor->AngularVelocity();
linearAcceleration = ImuSensor->LinearAcceleration();
body_quat = this->ImuSensor->Orientation();

ignition::math::Matrix3d m(body_quat);
body_rotation_matrix << m(0, 0), m(0, 1), m(0, 2),
                        m(1, 0), m(1, 1), m(1, 2),
                        m(2, 0), m(2, 1), m(2, 2);
body_roll = body_quat.Euler()[0];
body_pitch = body_quat.Euler()[1];



}

double pioneer::Turn(double t){
  	double X = TurningTrajectory_coeff(0) + TurningTrajectory_coeff(1) * t + TurningTrajectory_coeff(2) * pow(t, 2) + TurningTrajectory_coeff(3) * pow(t, 3) + TurningTrajectory_coeff(4) * pow(t, 4) + TurningTrajectory_coeff(5) * pow(t, 5);
	return X;
}
double pioneer::Back(double t){
  	double X = BackTrajectory_coeff(0) + BackTrajectory_coeff(1) * t + BackTrajectory_coeff(2) * pow(t, 2) + BackTrajectory_coeff(3) * pow(t, 3) + BackTrajectory_coeff(4) * pow(t, 4) + BackTrajectory_coeff(5) * pow(t, 5);
	return X;
}

// void pioneer::MakeMatlabFile(){
// theta_count += 1;
// fprintf(all_theta_data, "%d ", theta_count);
// for (int i = 0; i < 11; i++)
// {
//     fprintf(all_theta_data, "%lf ", sensor_th[i]);
// }
// fprintf(all_theta_data, "%lf\n", sensor_th[11]);

// fprintf(ref_theta_data, "%d ", theta_count);
// fprintf(ref_theta_data,"%lf  %lf  %lf  %lf\n",ref_RL_th(indext,0),ref_LL_th(indext,0),ref_RL_th(indext,3),ref_LL_th(indext,3));

// }


}