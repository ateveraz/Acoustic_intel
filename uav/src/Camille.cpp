//  created:    2024/04/19
//  filename:   Camille.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    
//
//
/*********************************************************************/

#include "Camille.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <ComboBox.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <CheckBox.h>
#include <Vector2DSpinBox.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <TrajectoryGenerator1D.h>
#include <TcpSocket.h>
#include <stdexcept>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

Camille::Camille(TargetController *controller,string ugvName,uint16_t listeningPort): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
	Uav* uav=GetUav();

	VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
	uavVrpn = new MetaVrpnObject(uav->ObjectName());
	targetVrpn=new MetaVrpnObject(ugvName);
	
	getFrameworkManager()->AddDeviceToLog(uavVrpn);
	getFrameworkManager()->AddDeviceToLog(targetVrpn);
	vrpnclient->Start();
	
	uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);
  	uav->GetAhrs()->YawPlot()->AddCurve(targetVrpn->State()->Element(2),DataPlot::Black);
															 
  	takeOffInPositionHold = new CheckBox(GetButtonsLayout()->NewRow(), "take off in position hold (and use z from optitrack)");
	manualZVRPN=new PushButton(GetButtonsLayout()->NewRow(),"manualZVRPN");
	positionHold=new PushButton(GetButtonsLayout()->LastRowLastCol(),"positionHold");
	carFollowing=new PushButton(GetButtonsLayout()->LastRowLastCol(),"CarFollowing");
	gotoGcsPosition=new PushButton(GetButtonsLayout()->NewRow(),"gotoGcsPosition");
	position=new Vector2DSpinBox(GetButtonsLayout()->LastRowLastCol(),"position",-5,5,1);


	// Define yaw regulation. 
	yawSettings = new GroupBox(GetButtonsLayout()->LastRowLastCol(), "Yaw settings");
	yawBehavior = new ComboBox(yawSettings->NewRow(), "Select yaw behavior");
	yawBehavior->AddItem("Set to zero (default)");
	yawBehavior->AddItem("Use yaw from Socket ROS");
	yawBehavior->AddItem("Use yaw from GUI");
	yawByGui = new DoubleSpinBox(yawSettings->NewRow(), "Yaw from GUI", -3.1416, 3.1416, 0.1, 4);

	gotoSocketPosition=new PushButton(GetButtonsLayout()->NewRow(),"gotoSocketPosition");
	safeLand=new Vector2DSpinBox(GetButtonsLayout()->NewRow(),"safe landing position",-10,10,1);

	// Simulate path_planning source
	planning_settings = new GroupBox(GetButtonsLayout()->NewRow(), "Path planning settings");
	findSource = new PushButton(planning_settings->NewRow(), "Find source");
	step_size = new DoubleSpinBox(planning_settings->NewRow(), "Step size", 0.01, 10, 0.1, 2);

	uX=new Pid(setupLawTab->At(1,0),"u_x");
	uX->UseDefaultPlot(graphLawTab->NewRow());
	uY=new Pid(setupLawTab->At(1,1),"u_y");
	uY->UseDefaultPlot(graphLawTab->LastRowLastCol());

	customReferenceOrientation= new AhrsData(this,"reference");
	uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
	AddDataToControlLawLog(customReferenceOrientation);
	AddDeviceToControlLawLog(uX);
	AddDeviceToControlLawLog(uY);

	customOrientation=new AhrsData(this,"orientation");
  
  	uavVrpn->zPlot()->AddCurve(GetAltitudeTrajectory()->GetMatrix()->Element(0), DataPlot::Green);
  	uavVrpn->VzPlot()->AddCurve(GetAltitudeTrajectory()->GetMatrix()->Element(1), DataPlot::Green);
	
	listeningSocket=new TcpSocket(uav,"Message",false,false);
	listeningSocket->Listen(listeningPort);
  Thread::Info("Debug: Listening to port %d\n", listeningPort);
  /*
  // accept incoming connection
  while (!message) {
    try {
      message = listeningSocket->Accept(100000000);
    } catch (std::logic_error &e) {
      //Thread::Err("%s\n",e.what());
      //return;
    } catch (std::runtime_error e) {
      // timeout
      //Thread::Err("%s\n",e.what());
      //return;
    }
  }
  Thread::Info("Debug: Connexion accepted\n");*/
}

Camille::~Camille() {
}

const AhrsData *Camille::GetOrientation(void) const {
	if (behaviourMode==BehaviourMode_t::ManualZVRPN) {
		return GetDefaultOrientation();
	} else {
		//get yaw from vrpn
		Quaternion vrpnQuaternion;
		uavVrpn->GetQuaternion(vrpnQuaternion);

		//get roll, pitch and w from imu
		Quaternion ahrsQuaternion;
		Vector3Df ahrsAngularSpeed;
		GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);
		
		Euler ahrsEuler=ahrsQuaternion.ToEuler();
		ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
		Quaternion mixQuaternion=ahrsEuler.ToQuaternion();

		customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

		return customOrientation;
	}
}

void Camille::AltitudeValues(float &z,float &dz) const{
	Vector3Df uav_pos,uav_vel;

	uavVrpn->GetPosition(uav_pos);
	uavVrpn->GetSpeed(uav_vel);
	//z and dz must be in uav's frame
	z=-uav_pos.z;
	dz=-uav_vel.z;
}

AhrsData *Camille::GetReferenceOrientation(void) {
	if (behaviourMode==BehaviourMode_t::ManualZVRPN) {
		customReferenceOrientation=(AhrsData*)GetDefaultReferenceOrientation();
	} else {
		Vector2Df pos_err, vel_err; // in Uav coordinate system
		float yaw_ref;
		Euler refAngles;
	
		PositionValues(pos_err, vel_err, yaw_ref);

		refAngles.yaw=yaw_ref;

		uX->SetValues(pos_err.x, vel_err.x);
		uX->Update(GetTime());
		refAngles.pitch=uX->Output();

		uY->SetValues(pos_err.y, vel_err.y);
		uY->Update(GetTime());
		refAngles.roll=-uY->Output();
		
		//TODO: in car following mode we could add car's wz
		customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));
	}

	return customReferenceOrientation;
}

void Camille::PositionValues(Vector2Df &pos_error,Vector2Df &vel_error,float &yaw_ref) {
	Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
	Vector2Df uav_2Dpos,uav_2Dvel; // in VRPN coordinate system

	uavVrpn->GetPosition(uav_pos);
	uavVrpn->GetSpeed(uav_vel);

	uav_pos.To2Dxy(uav_2Dpos);
	uav_vel.To2Dxy(uav_2Dvel);

	// In VRPN frame
	Quaternion currentQuaternion=GetCurrentQuaternion();
	Euler currentAngles;//in vrpn frame
	currentQuaternion.ToEuler(currentAngles);

	if (yawBehavior->CurrentIndex()==0) {
		yawDesired = 0;
	} else if (yawBehavior->CurrentIndex()==1) {
		yawDesired = yawFromSocket;
	} else if (yawBehavior->CurrentIndex()==2) {
		yawDesired = yawByGui->Value();
	} else {
		Thread::Err("yawBehavior not handled\n");
		EnterFailSafeMode();
	}

	if (behaviourMode==BehaviourMode_t::PositionHold) {
		pos_error=uav_2Dpos-posHold;
		vel_error=uav_2Dvel;
		yaw_ref=yawHold;
	} else if (behaviourMode==BehaviourMode_t::GotoGCSPosition){
		pos_error=uav_2Dpos-position->Value();
		vel_error=uav_2Dvel;
		yaw_ref=yawDesired;
	} else if (behaviourMode==BehaviourMode_t::GotoSocketPosition){
		pos_error=uav_2Dpos-socketPos;
		vel_error=uav_2Dvel;
		yaw_ref=yawDesired;
	} 
	else if (behaviourMode==BehaviourMode_t::GotoSourceUsingPathPlanning){
		float angle;
		if (yawBehavior->CurrentIndex()==1) {
			angle = yawFromSocket;
		} else {
			Vector3Df target_pos_sim;
			Vector2Df target_2Dpos_sim;
			targetVrpn->GetPosition(target_pos_sim);
			target_pos_sim.To2Dxy(target_2Dpos_sim);
	
			angle = atan2(target_2Dpos_sim.y - uav_2Dpos.y, target_2Dpos_sim.x - uav_2Dpos.x);
		}

		Vector2Df next_position;
		computePathPlannig(uav_2Dpos, angle, step_size->Value(), next_position);

		pos_error = uav_2Dpos-next_position;
		vel_error = uav_2Dvel;
		yaw_ref = angle;
	}
	else if (behaviourMode==BehaviourMode_t::CarFollowing){
		Vector3Df target_pos,target_vel;
		Vector2Df target_2Dpos,target_2Dvel;

		targetVrpn->GetPosition(target_pos);
		targetVrpn->GetSpeed(target_vel);
		target_pos.To2Dxy(target_2Dpos);
		target_vel.To2Dxy(target_2Dvel);
	 
		//error in optitrack frame
		pos_error=uav_2Dpos-target_2Dpos;
		vel_error=uav_2Dvel;//-target_2Dvel;
		Quaternion targetQuaternion;
		targetVrpn->GetQuaternion(targetQuaternion);
		yaw_ref=targetQuaternion.ToEuler().yaw;
	} else if (behaviourMode==BehaviourMode_t::LowBatteryGotoLandingPosition || behaviourMode==BehaviourMode_t::LowBatteryLanding){
		pos_error=uav_2Dpos-safeLand->Value();
		vel_error=uav_2Dvel;
		yaw_ref=yawHold;
		if(pos_error.GetNorm()<0.2) {
			Land();
			behaviourMode=BehaviourMode_t::LowBatteryLanding;
		}
	} else {
		Thread::Err("unhandled mode\n");
		EnterFailSafeMode();
	}

	//error in uav frame
	pos_error.Rotate(-currentAngles.yaw);
	vel_error.Rotate(-currentAngles.yaw);
}

void Camille::SignalEvent(Event_t event) {
	UavStateMachine::SignalEvent(event);
	switch(event) {
	case Event_t::TakingOff:
    if(takeOffInPositionHold->IsChecked()) {
      Printf("taking off in position hold mode\n");
      PositionHold();
    } else {
      Printf("taking off in manual mode\n");
      behaviourMode=BehaviourMode_t::Default;
    }
		vrpnLost=false;
		break;
	case Event_t::EnteringControlLoop:
    if (!message) {
      try {
        message = listeningSocket->Accept(TIME_NONBLOCK);
      } catch (std::logic_error &e) {
        //Thread::Err("%s\n",e.what());
        //return;
      } catch (std::runtime_error e) {
        // timeout
        //Thread::Err("%s\n",e.what());
        //return;
      }
      if (message) Thread::Info("Debug: Connexion accepted\n");
    }
		CheckMessages();
		break;
	case Event_t::EnteringFailSafeMode:
		behaviourMode=BehaviourMode_t::Default;
		break;
	}
}

void Camille::LowBatteryAction(void) {
	if (behaviourMode!=BehaviourMode_t::Default) {
		Quaternion vrpnQuaternion;
		uavVrpn->GetQuaternion(vrpnQuaternion);
		yawHold=vrpnQuaternion.ToEuler().yaw;

		uX->Reset();
		uY->Reset();
		behaviourMode=BehaviourMode_t::LowBatteryGotoLandingPosition;
		Thread::Info("Camille: LowBatteryGotoLandingPosition\n");
	} else {
		LowBatteryDefaultAction();
	}
}

void Camille::ExtraSecurityCheck(void) {
	if (!vrpnLost && behaviourMode!=BehaviourMode_t::Default) {
		if (!targetVrpn->IsTracked(500) && (behaviourMode==BehaviourMode_t::CarFollowing || behaviourMode==BehaviourMode_t::GotoSourceUsingPathPlanning)) {
			Thread::Err("VRPN, target lost\n");
			vrpnLost=true;
			EnterFailSafeMode();
			Land();
		}
		if (!uavVrpn->IsTracked(500)) {
			Thread::Err("VRPN, uav lost\n");
			vrpnLost=true;
			EnterFailSafeMode();
			Land();
		}
	}
}

void Camille::ExtraCheckPushButton(void) {
	if(manualZVRPN->Clicked()) {
		ManualZVRPN();
	}
	if(gotoGcsPosition->Clicked()) {
		GotoGcsPosition();
	}
	if(gotoSocketPosition->Clicked()) {
		GotoSocketPosition();
	}
	if(positionHold->Clicked()) {
		PositionHold();
	}
	if(carFollowing->Clicked()) {
		CarFollowing();
	}
	if(findSource->Clicked()) {
		GotoSourceUsingPathPlanning();
	}
}

void Camille::ExtraCheckJoystick(void) {
	//R1+Circle
	if(GetTargetController()->ButtonClicked(4) && GetTargetController()->IsButtonPressed(9)) {
		ManualZVRPN();
	}

	//R1+Cross
	if(GetTargetController()->ButtonClicked(5) && GetTargetController()->IsButtonPressed(9)) {
		GotoGcsPosition();
	}
	
	//R1+Square
	if(GetTargetController()->ButtonClicked(2) && GetTargetController()->IsButtonPressed(9)) {
		PositionHold();
	}
	
	//R1+Triangle
	if(GetTargetController()->ButtonClicked(3) && GetTargetController()->IsButtonPressed(9)) {
		CarFollowing();
	}
	//todo add a shortcut for goto socket position
}

void Camille::CheckMessages(void) {
  if(message) {
    float msg[4];
    while(message->RecvMessage((char*)msg,sizeof(msg),TIME_NONBLOCK)>0) {
			if(socketPos.x!=msg[0] || socketPos.y!=msg[1]) Printf("new socket pos %f %f\n",msg[0],msg[1]);
			socketPos.x=msg[0];
			socketPos.y=msg[1];
			// msg[2] contains the z-coordinate, but we don't use it. 
			yawFromSocket=msg[3];
    }
  }
}

void Camille::ManualZVRPN(void) {
	if( behaviourMode==BehaviourMode_t::ManualZVRPN) {
		Thread::Warn("Camille: already in ManualZVRPN mode\n");
		return;
	}
	if (!SetOrientationMode(OrientationMode_t::Custom)) {
		Thread::Warn("Camille: could not start ManualZVRPN mode\n");
		return;
	}
	behaviourMode=BehaviourMode_t::ManualZVRPN;
	Thread::Info("Camille: ManualZVRPN mode\n");
}

void Camille::GotoGcsPosition(void) {
	if( behaviourMode==BehaviourMode_t::GotoGCSPosition) {
		Thread::Warn("Camille: already in goto gcs position mode\n");
		return;
	}
	if (!SetOrientationMode(OrientationMode_t::Custom)) {
		Thread::Warn("Camille: could not goto gcs position mode\n");
		return;
	}
	
	GotoPosition();
	behaviourMode=BehaviourMode_t::GotoGCSPosition;
	Thread::Info("Camille: goto gcs position\n");
}

void Camille::GotoSocketPosition(void) {
	if( behaviourMode==BehaviourMode_t::GotoSocketPosition) {
		Thread::Warn("Camille: already in goto socket position mode\n");
		return;
	}
	if (!SetOrientationMode(OrientationMode_t::Custom)) {
		Thread::Warn("Camille: could not goto socket position mode\n");
		return;
	}
	
	GotoPosition();
	//use current pos as inital value
	Vector3Df uavPos;
	uavVrpn->GetPosition(uavPos);
	socketPos=uavPos.To2Dxy();
	
	behaviourMode=BehaviourMode_t::GotoSocketPosition;
	Thread::Info("Camille: goto socket position\n");
}

void Camille::GotoSourceUsingPathPlanning(void)
{
	behaviourMode=BehaviourMode_t::GotoSourceUsingPathPlanning;
	Thread::Info("Camille: goto source using path planning\n");
}

void Camille::computePathPlannig(Vector2Df uav_position, float angle, float step, Vector2Df &next_position)
{
	// Simulate path planning
	next_position = uav_position + step * Vector2Df(cos(angle) + sin(angle), sin(angle) - cos(angle));
}


void Camille::GotoPosition(void) {
	Quaternion vrpnQuaternion;
	uavVrpn->GetQuaternion(vrpnQuaternion);
	yawHold=vrpnQuaternion.ToEuler().yaw;

	uX->Reset();
	uY->Reset();
}

void Camille::PositionHold(void) {
	if( behaviourMode==BehaviourMode_t::PositionHold) {
		Thread::Warn("Camille: already in position hold mode\n");
		return;
	}
	if (!SetOrientationMode(OrientationMode_t::Custom)) {
		Thread::Warn("Camille: could not position hold mode\n");
		return;
	}
	
	Quaternion vrpnQuaternion;
	uavVrpn->GetQuaternion(vrpnQuaternion);
	yawHold=vrpnQuaternion.ToEuler().yaw;

	Vector3Df vrpnPosition;
	uavVrpn->GetPosition(vrpnPosition);
	vrpnPosition.To2Dxy(posHold);

	uX->Reset();
	uY->Reset();
	behaviourMode=BehaviourMode_t::PositionHold;
	Thread::Info("Camille: holding position\n");
}

void Camille::CarFollowing(void) {
	if( behaviourMode==BehaviourMode_t::CarFollowing) {
		Thread::Warn("Camille: already in CarFollowing mode\n");
		return;
	}
	if (!SetOrientationMode(OrientationMode_t::Custom)) {
		Thread::Warn("Camille: could not CarFollowing mode\n");
		return;
	}
	
	uX->Reset();
	uY->Reset();
	behaviourMode=BehaviourMode_t::CarFollowing;
	Thread::Info("Camille: CarFollowing\n");
}