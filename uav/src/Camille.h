//  created:    2024/04/19
//  filename:   Camille.h
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

#ifndef CAMILLE_H
#define CAMILLE_H

#include <UavStateMachine.h>

namespace flair {
    namespace gui {
        class PushButton;
        class Vector2DSpinBox;
        class CheckBox;
        class ComboBox;
        class DoubleSpinBox;
        class GroupBox;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
		namespace core {
        class TcpSocket;
    }
}

class Camille : public flair::meta::UavStateMachine {
    public:
        Camille(flair::sensor::TargetController *controller,std::string ugvName,uint16_t listeningPort);
        ~Camille();

    private:

	enum class BehaviourMode_t {
            Default,
						ManualZVRPN,
            PositionHold,
            GotoGCSPosition,
            GotoSocketPosition,
						CarFollowing,
						LowBatteryGotoLandingPosition,
						LowBatteryLanding,
            GotoSourceUsingPathPlanning,
        };

        BehaviourMode_t behaviourMode;
        bool vrpnLost;

        void ManualZVRPN(void);
        void GotoGcsPosition(void);
        void GotoSocketPosition(void);
        void GotoPosition(void);
        void PositionHold(void);
        void CarFollowing(void);
        void ExtraSecurityCheck(void);
        void ExtraCheckPushButton(void);
        void ExtraCheckJoystick(void);
        const flair::core::AhrsData *GetOrientation(void) const;
        void AltitudeValues(float &z,float &dz) const;
        void PositionValues(flair::core::Vector2Df &pos_error,flair::core::Vector2Df &vel_error,float &yaw_ref);
        flair::core::AhrsData *GetReferenceOrientation(void);
        void SignalEvent(Event_t event);
				void LowBatteryAction(void);
				void CheckMessages(void);
        void GotoSourceUsingPathPlanning(void);
        void computePathPlannig(flair::core::Vector2Df uav_position, float angle, float step, flair::core::Vector2Df &next_position);

        flair::filter::Pid *uX, *uY;

        flair::core::Vector2Df posHold;
        flair::core::Vector2Df socketPos;
        float yawHold;
        float yawDesired;
        float yawFromSocket;

        flair::gui::PushButton *manualZVRPN,*positionHold,*gotoGcsPosition,*gotoSocketPosition,*carFollowing, *findSource;
        flair::gui::CheckBox *takeOffInPositionHold;
        flair::gui::Vector2DSpinBox *position,*safeLand;
        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
				flair::core::TcpSocket *listeningSocket,*message= nullptr;

        flair::gui::GroupBox *yawSettings, *planning_settings;
        flair::gui::ComboBox *yawBehavior;
        flair::gui::DoubleSpinBox *yawByGui, *step_size;
};

#endif // CAMILLE_H
