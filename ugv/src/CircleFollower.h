//  created:    2020/12/09
//  filename:   CircleFollower.h
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

#ifndef CIRCLEFOLLOWER_H
#define CIRCLEFOLLOWER_H

#include <Thread.h>

namespace flair {
    namespace gui {
        class PushButton;
        class DoubleSpinBox;
    }
    namespace filter {
        class TrajectoryGenerator2DCircle;
        class Pid;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
}

class CircleFollower : public flair::core::Thread {
    public:
        CircleFollower(std::string name,flair::sensor::TargetController *controller);
        ~CircleFollower();

    private:

	enum class BehaviourMode_t {
            Manual,
            Circle
        };

        void Run(void);
        void StartCircle(void);
        void StopCircle(void);
        void ComputeManualControls(void);
        void ComputeCircleControls(void);
        void SecurityCheck(void);
        void CheckJoystick(void);
        void CheckPushButton(void);

        flair::filter::Pid *uX, *uY;
        flair::gui::PushButton *startCircle,*stopCircle,*quitProgram,*startLog,*stopLog;
        flair::gui::DoubleSpinBox *l;
        flair::meta::MetaVrpnObject *ugvVrpn;
        flair::filter::TrajectoryGenerator2DCircle *circle;
        BehaviourMode_t behaviourMode;
        bool vrpnLost;
        flair::sensor::TargetController *controller;
};

#endif // CIRCLEFOLLOWER_H
