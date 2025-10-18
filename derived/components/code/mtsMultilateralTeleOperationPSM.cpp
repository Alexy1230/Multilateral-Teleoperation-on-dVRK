/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Brendan Burkhart
  Created on: 2025-01-23

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

// header
#include "mtsMultilateralTeleOperationPSM.h"

// cisst includes
#include <cisstMultiTask/mtsManagerLocal.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsMultilateralTeleOperationPSM,
                                      mtsTeleOperationPSM,
                                      mtsTaskPeriodicConstructorArg);

void mtsMultilateralTeleOperationPSM::ForceSource::Configure(mtsMultilateralTeleOperationPSM* teleop, const Json::Value & jsonConfig) {
    std::string component_name;
    std::string provided_interface_name;
    std::string function_name;

    Json::Value value;

    value = jsonConfig["component"];
    if (!value.empty()) {
        component_name = value.asString();
    }

    value = jsonConfig["interface"];
    if (!value.empty()) {
        provided_interface_name = value.asString();
    }

    value = jsonConfig["function"];
    if (!value.empty()) {
        function_name = value.asString();
    }

    std::string required_interface_name = provided_interface_name + "-force-source";
    mtsInterfaceRequired* interface = teleop->AddInterfaceRequired(required_interface_name);
    if (interface) {
        interface->AddFunction(function_name, measured_cf);
    }

    mtsManagerLocal* manager = mtsComponentManager::GetInstance();
    manager->Connect(component_name, provided_interface_name,
                     teleop->GetName(), required_interface_name);
}

// why need force source?  measured_cf? <-> measured_cs?
// why measured_cf and m_measured_cf

void mtsMultilateralTeleOperationPSM::Arm::populateInterface(mtsInterfaceRequired* interface)
{
    interface->AddFunction("servo_cs", servo_cs, MTS_OPTIONAL);
    interface->AddFunction("measured_cs", measured_cs, MTS_OPTIONAL);
}


prmStateCartesian mtsMultilateralTeleOperationPSM::ArmPSM::computeGoal(Arm* target1, Arm* target2, double scale)
{
    prmStateCartesian goal;
    prmStateCartesian target1_state = target1->state();
    prmStateCartesian target2_state = target2->state();

    vct3 target1_translation = target1_state.Position().Translation() - target2->ClutchOrigin().Translation();
    vct3 target2_translation = target2_state.Position().Translation() - target1->ClutchOrigin().Translation();
    vct3 goal_translation = scale * (0.5 * target1_translation + 0.5 * target2_translation) + ClutchOrigin().Translation();

    auto align_offset_target1 = vctMatRot3(target1->ClutchOrigin().Rotation().TransposeRef() * ClutchOrigin().Rotation());
    auto align_offset_target2 = vctMatRot3(target2->ClutchOrigin().Rotation().TransposeRef() * ClutchOrigin().Rotation());
    auto goal_target1 = target1_state.Position().Rotation() * align_offset_target1;
    auto goal_target2 = target2_state.Position().Rotation() * align_offset_target2;
    
    // compute average of two rotations
    vctQuatRot3 goal_quat1(goal_target1.Rotation(), true);
    vctQuatRot3 goal_quat2(goal_target2.Rotation(), true);
    vctQuatRot3 goal_quat_aver(0.5 * (goal_quat1.X()+goal_quat2.X()), 0.5 * (goal_quat1.Y()+goal_quat2.Y()), 
                               0.5 * (goal_quat1.Z()+goal_quat2.Z()), 0.5 * (goal_quat1.R()+goal_quat2.R()), true);
    vctMatRot3 goal_rotation = vctMatRot3(goal_quat_aver);

    if (target1_state.PositionIsValid() && target2_state.PositionIsValid()) {
        // translation goal
        goal.Position().Translation() = goal_translation;
        // rotation goal
        goal.Position().Rotation() = goal_rotation;
    }
    goal.PositionIsValid() = target1_state.PositionIsValid() && target2_state.PositionIsValid();

    if (target1_state.VelocityIsValid() && target2_state.VelocityIsValid()) {

        goal.Velocity().Ref<3>(0) = 0.5 * scale * target1_state.Velocity().Ref<3>(0) + 0.5 * scale * target2_state.Velocity().Ref<3>(0);
        goal.Velocity().Ref<3>(3) = 0.5 * target1_state.Velocity().Ref<3>(3) + 0.5 * target2_state.Velocity().Ref<3>(3);
    }
    goal.VelocityIsValid() = target1_state.VelocityIsValid() && target2_state.VelocityIsValid()

    prmStateCartesian current_state = state();
    if (target1_state.ForceIsValid() && target2_state.ForceIsValid() && current_state.ForceIsValid()) {
        goal.Force() = -0.5 * target1_state.Force() - 0.5 * target2_state.Force() - current_state.Force();
    }
    goal.ForceIsValid() = target1_state.ForceIsValid() && target2_state.ForceIsValid() && current_state.ForceIsValid();

    return goal;
}


prmStateCartesian mtsMultilateralTeleOperationPSM::ArmMTM::computeGoal(Arm* target_PSM, Arm* target_MTM, double scale)
{
    prmStateCartesian goal;
    prmStateCartesian target_PSM_state = target_PSM->state();
    prmStateCartesian target_MTM_state = target_MTM->state();

    vct3 target_PSM_translation = target_PSM_state.Position().Translation() - target_PSM->ClutchOrigin().Translation();
    vct3 goal_translation = target_PSM_translation / scale + ClutchOrigin().Translation();

    auto align = vctMatRot3(target_PSM->ClutchOrigin().Rotation().TransposeRef() * ClutchOrigin().Rotation());

    if (target_PSM_state.PositionIsValid()) {
        goal.Position().Translation() = goal_translation;
        goal.Position().Rotation() = target_PSM_state.Position().Rotation() * align;
    }
    goal.PositionIsValid() = target_PSM_state.PositionIsValid();

    if (target_PSM_state.VelocityIsValid()) {
        goal.Velocity().Ref<3>(0) = target_PSM_state.Velocity().Ref<3>(0) / scale;
        goal.Velocity().Ref<3>(3) = target_PSM_state.Velocity().Ref<3>(3);
    }
    goal.VelocityIsValid() = target_PSM_state.VelocityIsValid();

    prmStateCartesian current_state = state();
    if (target_PSM_state.ForceIsValid() && target_MTM_state.ForceIsValid() && current_state.ForceIsValid()) {
        goal.Force() = -target_PSM_state.Force() - 0.5 * target_MTM_state.Force() - 0.5 * current_state.Force();
    }
    goal.ForceIsValid() = target_PSM_state.ForceIsValid() && target_MTM_state.ForceIsValid() && current_state.ForceIsValid();

    return goal;
}


prmStateCartesian mtsMultilateralTeleOperationPSM::Arm::state()
{
    prmStateCartesian measured_state;
    measured_cs(measured_state);

    if (force_source) {
        force_source->measured_cf(force_source->m_measured_cf);
        measured_state.Force() = force_source->m_measured_cf.Force();
        measured_state.ForceIsValid() = force_source->m_measured_cf.Valid();
    }

    return measured_state;
}

void mtsMultilateralTeleOperationPSM::Arm::servo(prmStateCartesian goal)
{
    servo_cs(goal);
}

vctFrm4x4& mtsMultilateralTeleOperationPSM::ArmMTM::ClutchOrigin() { return teleop->mMTM.CartesianInitial; }

prmStateCartesian mtsMultilateralTeleOperationPSM::ArmMTM::state()
{
    if (measured_cs.IsValid()) {
        return Arm::state();
    }

    // measured_cs not available, fall back to measured_cp/measured_cv
    prmStateCartesian state;
    state.Position() = teleop->mMTM.m_measured_cp.Position();
    state.PositionIsValid() = teleop->mMTM.m_measured_cp.Valid();

    if (teleop->m_config.use_MTM_velocity) {
        auto mtm_velocity = teleop->mMTM.m_measured_cv;
        state.Velocity().Ref<3>(0) = mtm_velocity.VelocityLinear();
        state.Velocity().Ref<3>(3) = mtm_velocity.VelocityAngular();
        state.VelocityIsValid() = mtm_velocity.Valid();
    } else {
        state.VelocityIsValid() = false;
    }

    state.ForceIsValid() = false;

    return state;
}

void mtsMultilateralTeleOperationPSM::ArmMTM::servo(prmStateCartesian goal)
{
    // Use servo_cs if available, otherwise fall back to servo_cp
    if (servo_cs.IsValid()) {
        Arm::servo(goal);
    } else {
        prmPositionCartesianSet& servo = teleop->mArmMTM.m_servo_cp;
        servo.Goal() = goal.Position();

        if (goal.VelocityIsValid()) {
            servo.Velocity() = goal.Velocity().Ref<3>(0);
            servo.VelocityAngular() = goal.Velocity().Ref<3>(3);
        } else {
            servo.Velocity().Assign(vct3(0));
            servo.VelocityAngular().Assign(vct3(0));
        }

        teleop->mArmMTM.servo_cp(servo);
    }
}

vctFrm4x4& mtsMultilateralTeleOperationPSM::ArmPSM::ClutchOrigin() { return teleop->mPSM.CartesianInitial; };

prmStateCartesian mtsMultilateralTeleOperationPSM::ArmPSM::state()
{
    if (measured_cs.IsValid()) {
        return Arm::state();
    }

    // measured_cs not available, fall back to measured_cp/measured_cv
    
    prmStateCartesian state;
    teleop->mArmPSM.measured_cp(teleop->mArmPSM.m_measured_cp);
    state.Position() = teleop->mArmPSM.m_measured_cp.Position();
    state.PositionIsValid() = teleop->mArmPSM.m_measured_cp.Valid();

    teleop->mArmPSM.measured_cv(teleop->mArmPSM.m_measured_cv);
    auto psm_velocity = teleop->mArmPSM.m_measured_cv;
    state.Velocity().Ref<3>(0) = psm_velocity.VelocityLinear();
    state.Velocity().Ref<3>(3) = psm_velocity.VelocityAngular();
    state.VelocityIsValid() = psm_velocity.Valid();

    state.ForceIsValid() = false;

    return state;
}

void mtsMultilateralTeleOperationPSM::ArmPSM::servo(prmStateCartesian goal)
{
    // Use servo_cs if available, otherwise fall back to servo_cp
    if (servo_cs.IsValid()) {
        Arm::servo(goal);
    } else {
        prmPositionCartesianSet& servo = teleop->mPSM.m_servo_cp;
        servo.Goal() = goal.Position();

        if (goal.VelocityIsValid()) {
            servo.Velocity() = goal.Velocity().Ref<3>(0);
            servo.VelocityAngular() = goal.Velocity().Ref<3>(3);
        } else {
            servo.Velocity().Assign(vct3(0));
            servo.VelocityAngular().Assign(vct3(0));
        }

        teleop->mPSM.servo_cp(servo);
    }
}

mtsMultilateralTeleOperationPSM::mtsMultilateralTeleOperationPSM(const std::string & componentName,
                                                       const double periodInSeconds) :
    mtsTeleOperationPSM(componentName, periodInSeconds), mArmMTM(this), mArmPSM(this) { Init(); }

mtsMultilateralTeleOperationPSM::mtsMultilateralTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg) :
    mtsTeleOperationPSM(arg), mArmMTM(this), mArmPSM(this) { Init(); }

void mtsMultilateralTeleOperationPSM::Init() {
    m_Multilateral_enabled = true;

    mtsInterfaceRequired* interface;

    interface = GetInterfaceRequired("MTM");
    if (interface) {
        interface->AddFunction("servo_cp", mArmMTM.servo_cp);
        mArmMTM.populateInterface(interface);
    }

    interface = GetInterfaceRequired("PSM");
    if (interface) {
        interface->AddFunction("measured_cp", mArmPSM.measured_cp);
        mArmPSM.populateInterface(interface);
    }

    mConfigurationStateTable->AddData(m_Multilateral_enabled, "Multilateral_enabled");

    mtsInterfaceProvided* setting_interface = GetInterfaceProvided("Setting");
    if (setting_interface) {
        setting_interface->AddCommandWrite(&mtsMultilateralTeleOperationPSM::set_Multilateral_enabled, this,
                                        "set_Multilateral_enabled", m_Multilateral_enabled);
        setting_interface->AddCommandReadState(*(mConfigurationStateTable),
                                        m_Multilateral_enabled, "Multilateral_enabled");
        setting_interface->AddEventWrite(Multilateral_enabled_event,
                                    "Multilateral_enabled", m_Multilateral_enabled);
    }
}

void mtsMultilateralTeleOperationPSM::Configure(const Json::Value & jsonConfig)
{
    mtsTeleOperationPSM::Configure(jsonConfig);
    Json::Value jsonValue;

    jsonValue = jsonConfig["psm_force_source"];
    if (!jsonValue.empty()) {
        auto source = std::make_unique<ForceSource>();
        source->Configure(this, jsonValue);
        mArmPSM.add_force_source(std::move(source));
    }

    jsonValue = jsonConfig["mtm1_force_source"];
    if (!jsonValue.empty()) {
        auto source = std::make_unique<ForceSource>();
        source->Configure(this, jsonValue);
        mArmMTM.add_force_source(std::move(source));
    }

    jsonValue = jsonConfig["mtm1_torque_gain"];
    if (!jsonValue.empty()) {
        m_mtm_torque_gain = jsonValue.asDouble();
    }

    jsonValue = jsonConfig["mtm2_force_source"];
    if (!jsonValue.empty()) {
        auto source = std::make_unique<ForceSource>();
        source->Configure(this, jsonValue);
        mArmMTM.add_force_source(std::move(source));
    }

    jsonValue = jsonConfig["mtm2_torque_gain"];
    if (!jsonValue.empty()) {
        m_mtm_torque_gain = jsonValue.asDouble();
    }
}

void mtsMultilateralTeleOperationPSM::set_Multilateral_enabled(const bool & enabled)
{
    mConfigurationStateTable->Start();
    m_Multilateral_enabled = enabled;
    mConfigurationStateTable->Advance();
    Multilateral_enabled_event(m_Multilateral_enabled);
}

void mtsMultilateralTeleOperationPSM::RunCartesianTeleop()
{
    // fall back to default behavior when in unilateral mode
    if (!m_Multilateral_enabled) {
        mtsTeleOperationPSM::RunCartesianTeleop();
        return;
    }

    if (m_clutched) {
        return;
    }

    auto psm_goal = mArmPSM.computeGoal(&mArmMTM, m_config.scale);
    mArmPSM.servo(psm_goal);

    auto mtm_goal = mArmMTM.computeGoal(&mArmPSM, 1.0 / m_config.scale);
    // scale MTM torque goal to reduce oscillations
    mtm_goal.Force().Ref<3>(3) = m_mtm_torque_gain * mtm_goal.Force().Ref<3>(3);
    mArmMTM.servo(mtm_goal);
}
