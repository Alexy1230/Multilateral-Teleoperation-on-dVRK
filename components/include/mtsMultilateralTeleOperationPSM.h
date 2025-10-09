/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Brendan Burkhart
  Created on: 2025-01-23

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsMultilateralTeleOperationPSM_h
#define _mtsMultilateralTeleOperationPSM_h

#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmStateCartesian.h>

class CISST_EXPORT mtsMultilateralTeleOperationPSM: public mtsTeleOperationPSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsMultilateralTeleOperationPSM(const std::string & componentName, const double periodInSeconds);
    mtsMultilateralTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsMultilateralTeleOperationPSM() {}

    void Configure(const Json::Value & jsonConfig) override;

    void set_Multilateral_enabled(const bool & enabled);

protected:
    class ForceSource {
    public:
        void Configure(mtsMultilateralTeleOperationPSM* teleop, const Json::Value & jsonConfig);

        mtsFunctionRead measured_cf;
        prmForceCartesianGet m_measured_cf;
    };

    class Arm {
    public:
        Arm(mtsMultilateralTeleOperationPSM* teleop) : teleop(teleop) {}
        virtual ~Arm() {};
        
        virtual void populateInterface(mtsInterfaceRequired* interface);
        virtual void add_force_source(std::unique_ptr<ForceSource> source) { force_source = std::move(source); }

        virtual prmStateCartesian computeGoal(Arm* target, double scale);

        virtual vctFrm4x4& ClutchOrigin() = 0;

        virtual prmStateCartesian state();
        virtual void servo(prmStateCartesian goal);

    protected:
        mtsMultilateralTeleOperationPSM* teleop;
        std::unique_ptr<ForceSource> force_source;

        mtsFunctionWrite servo_cpvf;
        mtsFunctionRead measured_cs;
    };

    class ArmMTM : public Arm {
    public:
        ArmMTM(mtsMultilateralTeleOperationPSM* teleop) : Arm(teleop) {}
        ~ArmMTM() {}

        vctFrm4x4& ClutchOrigin() override;
        
        prmStateCartesian state() override;
        void servo(prmStateCartesian goal) override;

        mtsFunctionWrite servo_cp;
        prmPositionCartesianSet m_servo_cp;
    };

    class ArmPSM : public Arm {
    public:
        ArmPSM(mtsMultilateralTeleOperationPSM* teleop) : Arm(teleop) {}
        ~ArmPSM() {}

        vctFrm4x4& ClutchOrigin() override;
        
        prmStateCartesian state() override;
        void servo(prmStateCartesian goal) override;

        mtsFunctionRead  measured_cp;
        mtsFunctionRead  measured_cv;
        prmPositionCartesianGet m_measured_cp;
        prmVelocityCartesianGet m_measured_cv;
    };

    ArmMTM mArmMTM;
    ArmPSM mArmPSM;

    bool m_Multilateral_enabled;
    mtsFunctionWrite Multilateral_enabled_event;

    double m_mtm_torque_gain = 0.2;

    void Init() override;

    void RunCartesianTeleop() override;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsMultilateralTeleOperationPSM);

#endif // _mtsBilateralTeleOperationPSM_h
