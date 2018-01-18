/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg04
    {
        /**
         * @brief [initialize_model.m]
         *
         * @todo AS. Currently the state of the swing foot is not updated
         * during the swing phase, i.e. it is the same as it was in the end of
         * the preceding TDS. It may be more reasonable to undefine this state
         * instead.
         */
        class HUMOTO_LOCAL Model :  public humoto::Model,
                                    public humoto::walking::PointMassModel6z,
                                    public humoto::walking::RobotFootParameters
        {

            public:
                // previous state of the model
                humoto::wpg04::ModelState       previous_state_;
                // state of the model
                humoto::wpg04::ModelState       state_;

                /// 2d position of the current support (center of a foot or ADS)
                etools::Vector2                current_support_position_;

                std::vector<boost::shared_ptr<humoto::obstacle_avoidance::ObstacleBase> > obstacles_;

            private:
                /**
                 * @brief Determines position of support.
                 */
                void determineSupportPosition()
                {
                    /// assume that in DS feet are aligned
                    switch(state_.stance_type_)
                    {
                        case humoto::walking::StanceType::LSS:
                            current_support_position_ = state_.feet_.getLeft().position_.head(2);
                            break;
                        case humoto::walking::StanceType::RSS:
                            current_support_position_ = state_.feet_.getRight().position_.head(2);
                            break;
                        case humoto::walking::StanceType::DS:
                            current_support_position_  = (state_.feet_.getRight().position_.head(2)
                                                            + state_.feet_.getLeft().position_.head(2)) / 2.;
                            break;
                        case humoto::walking::StanceType::TDS:
                            switch(state_.next_stance_type_)
                            {
                                case humoto::walking::StanceType::LSS:
                                    current_support_position_ = state_.feet_.getLeft().position_.head(2);
                                    break;
                                case humoto::walking::StanceType::RSS:
                                    current_support_position_ = state_.feet_.getRight().position_.head(2);
                                    break;
                                case humoto::walking::StanceType::DS:
                                    current_support_position_  = (state_.feet_.getRight().position_.head(2)
                                                                    + state_.feet_.getLeft().position_.head(2)) / 2.;
                                    break;
                                default:
                                    HUMOTO_THROW_MSG("Wrong state sequence.");
                                    break;
                            }
                            break;
                        default:
                            HUMOTO_THROW_MSG("Unknown stance type.");
                            break;
                    }
                }


            public:
                /**
                 * @brief Default constructor
                 */
                Model()
                {
                    determineSupportPosition();
                }


                /**
                 * @brief Add obstacle
                 *
                 * @param[in] obstacle
                 */
                void addObstacle(const boost::shared_ptr<humoto::obstacle_avoidance::ObstacleBase>& obstacle)
                {
                    obstacles_.push_back(obstacle);
                }


                /**
                 * @brief Update obstacles
                 *
                 * @param[in] control_problem
                 * @param[in] iteration_time
                 * @param[in] old_solution
                 * @param[in] safety_margin
                 */
                void updateObstacles(const humoto::ControlProblem& control_problem,
                                     const std::size_t             iteration_time,
                                     const humoto::Solution&       old_solution,
                                     const double                  safety_margin)
                {
                    // -------------------------------------------------- //

                    // if the instant 0 is passed, we can update
                    // the collision avoidance constratins.
                    // Update the matrices used
                    // for the CollAvoidance Jacobian

                    // First, I check how many obstacles appear around
                    // the robot (Assumed FoV). This will be used to initialize
                    // the current position of the obstacle as variable.

                    // For each obstacle we build a Jacobian matrix
                    // w.r.t. the conservative model of the obstacle
                    // considered.

                    if(iteration_time == 0)
                    {
                        for(std::size_t i = 0; i < obstacles_.size(); ++i)
                        {
                            obstacles_[i]->resetConstraints(control_problem);
                        }
                        return;
                    }

                    for(std::size_t i = 0; i < obstacles_.size(); ++i)
                    {
                        obstacles_[i]->updateConstraints(control_problem,
                                                         old_solution,
                                                         safety_margin);
                    }

                    // We stack together the matrices ready to be
                    // integrated in a compact constraint form (A,b)
                    // N constraints for each obstacle.

                    // Stack together:
                    // M: the condense position of the obstacles
                    // D: the distance vector (same vector for each obst.)
                    // J: the jacobian matrices
                    // stackTogetherCollAvoidanceMatrix();

                    // in order to make the constraints work
                    // the condense position of the CoM should be stack for
                    // each obstacle via a selection matrix.

                    // }
                    // -------------------------------------------------- //
                }


                /**
                 * @brief Constructor
                 *
                 * @param[in] foot_param parameters of the robot
                 */
                void setFootParameters(const humoto::walking::RobotFootParameters &foot_param)
                {
                    humoto::walking::RobotFootParameters::operator=(foot_param);
                }


                /**
                 * @brief Returns current CoM state.
                 *
                 * @return CoM state
                 */
                humoto::rigidbody::PointMassState   getCoMState() const
                {
                    return (state_.com_state_);
                }


                // Same as "getCoMState" but for "previous_state_"
                humoto::rigidbody::PointMassState   getPrevCoMState() const
                {
                    return (previous_state_.com_state_);
                }


                /**
                 * @brief Returns current foot state.
                 *
                 * @param[in] left_or_right left/right
                 *
                 * @return foot state
                 */
                humoto::rigidbody::RigidBodyState getFootState(const humoto::LeftOrRight::Type left_or_right) const
                {
                    return (state_.feet_[left_or_right]);
                }


                /**
                 * @brief Get cstate
                 */
                etools::Vector6 getCState() const
                {
                    // This only give me back a state with x and y position
                    // Cstate = [c_x dc_x ddc_x c_y dc_y ddc_y]'
                    etools::Vector6 cstate = convertCoMState(state_.com_state_);
                    return(cstate);
                }

                // Same as "getCState" but for "previous_state_"
                etools::Vector6 getPrevCState() const
                {
                    etools::Vector6 prev_cstate = convertCoMState(previous_state_.com_state_);
                    return(prev_cstate);
                }


                /**
                 * @brief Get CoM height
                 *
                 * @return CoM height
                 */
                double getCoMHeight() const
                {
                    return (state_.com_state_.position_.z());
                }

                // Same as "getCoMHeight" but for "previous_state_"
                double getPrevCoMHeight() const
                {
                    return (previous_state_.com_state_.position_.z());
                }

                /**
                 * @brief Update model state.
                 *
                 * @param[in] model_state model state.
                 */
                void    updateState(const humoto::ModelState &model_state)
                {

                    // Update the previous state
                    previous_state_.com_state_ = state_.com_state_;

                    const humoto::wpg04::ModelState &state = dynamic_cast <const humoto::wpg04::ModelState &> (model_state);

                    walking::StanceType::Type   prev_stance_type = state_.stance_type_;

                    state_.com_state_ = state.com_state_;

                    state_.stance_type_      = state.stance_type_;
                    state_.next_stance_type_ = state.next_stance_type_;


                    switch (state.stance_type_)
                    {
                        case humoto::walking::StanceType::LSS:
                            // XXX (undefined)
                            //state_.feet_.copyRight(state.feet_);
                            state_.feet_.copyLeft(state.feet_);
                            break;

                        case humoto::walking::StanceType::RSS:
                            // XXX (undefined)
                            //state_.feet_.copyLeft(state.feet_);
                            state_.feet_.copyRight(state.feet_);
                            break;

                        case humoto::walking::StanceType::TDS:
                            switch (prev_stance_type)
                            {
                                case humoto::walking::StanceType::LSS:
                                    state_.feet_.copyRight(state.feet_);
                                    break;

                                case humoto::walking::StanceType::RSS:
                                    state_.feet_.copyLeft(state.feet_);
                                    break;

                                case humoto::walking::StanceType::DS:
                                    // no need to update positions of the feet (they must not change)
                                    break;

                                default:
                                    HUMOTO_THROW_MSG("Unknown stance type.");
                                    break;
                            }
                            break;

                        case humoto::walking::StanceType::DS:
                            state_.feet_.copyLeft(state.feet_);
                            state_.feet_.copyRight(state.feet_);
                            break;

                        default:
                            HUMOTO_THROW_MSG("Unknown state type.");
                            break;
                    }

                    determineSupportPosition();
                }


                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "model") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    RobotFootParameters::log(logger, subname, "foot_parameters");

                    state_.log(logger, subname, "state");

                    logger.log(LogEntryName(subname).add("cstate"), getCState());
                    logger.log(LogEntryName(subname).add("current_support_position"), current_support_position_);
                }
        };
    }
}
