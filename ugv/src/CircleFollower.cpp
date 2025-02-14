//  created:    2020/12/09
//  filename:   CircleFollower.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#include "CircleFollower.h"
#include <TargetController.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Matrix.h>
#include <Tab.h>
#include <TabWidget.h>
#include <DoubleSpinBox.h>
#include <Pid.h>
#include <Quaternion.h>
#include <Euler.h>
#include <Ugv.h>
#include <UgvControls.h>
#include <math.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;
using namespace flair::actuator;

CircleFollower::CircleFollower(string name,TargetController *controller): Thread(getFrameworkManager(),"CircleFollower",50), behaviourMode(BehaviourMode_t::Manual), vrpnLost(false) {
    this->controller=controller;
    controller->Start();
    
    Ugv* ugv=GetUgv();
    ugv->UseDefaultPlot();
    
    VrpnClient* vrpnclient=new VrpnClient("vrpn", ugv->GetDefaultVrpnAddress(),80);
    ugvVrpn = new MetaVrpnObject(name);
    getFrameworkManager()->AddDeviceToLog(ugvVrpn);
    vrpnclient->Start();
	
    Tab *ugvTab = new Tab(getFrameworkManager()->GetTabWidget(), "ugv", 0);
    GridLayout* buttonslayout = new GridLayout(ugvTab->NewRow(), "buttons");
    quitProgram = new PushButton(buttonslayout->NewRow(), "quit program");
    startCircle=new PushButton(buttonslayout->NewRow(),"start_circle");
    stopCircle=new PushButton(buttonslayout->LastRowLastCol(),"stop_circle");
    startLog = new PushButton(buttonslayout->NewRow(), "start_log");
    stopLog = new PushButton(buttonslayout->LastRowLastCol(), "stop_log");
    
    circle=new TrajectoryGenerator2DCircle(vrpnclient->GetLayout()->NewRow(),"circle");
    ugvVrpn->xPlot()->AddCurve(circle->GetMatrix()->Element(0,0),DataPlot::Blue);
    ugvVrpn->yPlot()->AddCurve(circle->GetMatrix()->Element(0,1),DataPlot::Blue);
    ugvVrpn->VxPlot()->AddCurve(circle->GetMatrix()->Element(1,0),DataPlot::Blue);
    ugvVrpn->VyPlot()->AddCurve(circle->GetMatrix()->Element(1,1),DataPlot::Blue);
    ugvVrpn->XyPlot()->AddCurve(circle->GetMatrix()->Element(0,1),circle->GetMatrix()->Element(0,0),DataPlot::Blue,"circle");

    Tab *lawTab = new Tab(getFrameworkManager()->GetTabWidget(), "control laws");
    TabWidget *tabWidget = new TabWidget(lawTab->NewRow(), "laws");
    Tab *setupLawTab = new Tab(tabWidget, "Setup");
    Tab *graphLawTab = new Tab(tabWidget, "Graphes");
    uX=new Pid(setupLawTab->At(1,0),"u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY=new Pid(setupLawTab->At(1,1),"u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());
    
    getFrameworkManager()->AddDeviceToLog(uX);
    getFrameworkManager()->AddDeviceToLog(uY);
    
    l=new DoubleSpinBox(setupLawTab->NewRow(),"L", " m", 0, 10, 0.1, 1,1);
}

CircleFollower::~CircleFollower() {
}

void CircleFollower::Run(void) {
    WarnUponSwitches(true);
    SetPeriodMS(20);
    
    if (getFrameworkManager()->ErrorOccured() == true) {
        SafeStop();
    }

    while (!ToBeStopped()) {
        SecurityCheck();
        CheckJoystick();
        CheckPushButton();
       
        if(behaviourMode==BehaviourMode_t::Manual) ComputeManualControls();
        if(behaviourMode==BehaviourMode_t::Circle) ComputeCircleControls();
        WaitPeriod();
    }
}

void CircleFollower::CheckPushButton(void) {
  if (startLog->Clicked() == true)
    getFrameworkManager()->StartLog();
  if (stopLog->Clicked() == true)
    getFrameworkManager()->StopLog();
  
  if (startCircle->Clicked() == true)
      StartCircle();
        
  if (stopCircle->Clicked() == true)
      StopCircle();

  if (quitProgram->Clicked() == true)
      SafeStop();
}

void CircleFollower::CheckJoystick(void) {
  //R1 and Circle
  if(controller->ButtonClicked(4) && controller->IsButtonPressed(9)) {
      StartCircle();
  }

  //R1 and Cross
  if(controller->ButtonClicked(5) && controller->IsButtonPressed(9)) {
      StopCircle();
  }
}

void CircleFollower::SecurityCheck(void) {
    if ((!vrpnLost) && (behaviourMode==BehaviourMode_t::Circle)) {
        if (!ugvVrpn->IsTracked(500)) {
            Thread::Err("VRPN, ugv lost\n");
            vrpnLost=true;
            StopCircle();
        }
    }
}

void CircleFollower::ComputeManualControls(void) {
  float speed=-controller->GetAxisValue(3);
  float turn=controller->GetAxisValue(0);
  GetUgv()->GetUgvControls()->SetControls(speed,turn);
}

void CircleFollower::ComputeCircleControls(void) {

  Vector3Df ugv_pos,ugv_vel; // in VRPN coordinate system
  Vector2Df ugv_2Dpos,ugv_2Dvel; // in VRPN coordinate system
  Vector2Df pos_error,vel_error;
  Vector2Df circle_pos,circle_vel;
    
  ugvVrpn->GetPosition(ugv_pos);
  ugvVrpn->GetSpeed(ugv_vel);

  ugv_pos.To2Dxy(ugv_2Dpos);
  ugv_vel.To2Dxy(ugv_2Dvel);
  
  //circle reference
  circle->Update(GetTime());
  circle->GetPosition(circle_pos);
  circle->GetSpeed(circle_vel);

  //error in optitrack frame
  pos_error=ugv_2Dpos-circle_pos;
  vel_error=ugv_2Dvel-circle_vel;
    
  uX->SetValues(pos_error.x, vel_error.x);
  uX->Update(GetTime());
  uY->SetValues(pos_error.y, vel_error.y);
  uY->Update(GetTime());
  
  //get yaw from vrpn
  Quaternion vrpnQuaternion;
  ugvVrpn->GetQuaternion(vrpnQuaternion);
  float yaw=vrpnQuaternion.ToEuler().yaw;
  float v= cosf(yaw)*uX->Output() + sinf(yaw)*uY->Output();
  float w = -sinf(yaw)/l->Value()*uX->Output() + cosf(yaw)/l->Value()*uY->Output();
  GetUgv()->GetUgvControls()->SetControls(-v,-w);
}


void CircleFollower::StartCircle(void) {
  if(behaviourMode!=BehaviourMode_t::Circle) {
    Vector3Df ugv_pos;
    Vector2Df ugv_2Dpos,center_offset;
    
    //get yaw from vrpn
    Quaternion vrpnQuaternion;
    ugvVrpn->GetQuaternion(vrpnQuaternion);
    
    ugvVrpn->GetPosition(ugv_pos);
    ugv_pos.To2Dxy(ugv_2Dpos);
    //set circle center in front of UAV
    center_offset.x=circle->GetRadius();//center in UGV frame
    center_offset.y=0;
    center_offset.Rotate(vrpnQuaternion.ToEuler().yaw);//center in earth frame
    circle->SetCenter(ugv_2Dpos+center_offset);
    circle->StartTraj(ugv_2Dpos);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::Circle;
    Thread::Info("CircleFollower: start circle\n");
  }
}

void CircleFollower::StopCircle(void) {
	if(behaviourMode==BehaviourMode_t::Circle) {
    circle->FinishTraj();
    //GetJoystick()->Rumble(0x70);
    behaviourMode=BehaviourMode_t::Manual;
    Thread::Info("CircleFollower: finishing circle\n");
  }
}

