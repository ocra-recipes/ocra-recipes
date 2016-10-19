#ifndef MESSAGE_VOCABULARY_H
#define MESSAGE_VOCABULARY_H

#include <iostream>

namespace ocra_recipes
{

enum SERVER_COMMUNICATIONS_MESSAGE
{
    // General indicators
    FAILURE = 0,
    SUCCESS,
    WARNING,

    // Controller requests
    GET_CONTROLLER_STATUS,
    GET_WBI_CONFIG_FILE_PATH,
    GET_ROBOT_NAME,
    GET_IS_FLOATING_BASE,

    START_CONTROLLER,
    STOP_CONTROLLER,
    PAUSE_CONTROLLER,
    CHANGE_FIXED_LINK_RIGHT,
    CHANGE_FIXED_LINK_LEFT,
    FEET_CONTACT_STATE,

    // Controller status indicators
    CONTROLLER_RUNNING,
    CONTROLLER_STOPPED,
    CONTROLLER_PAUSED,

    // Task requests
    ADD_TASKS,
    ADD_TASKS_FROM_FILE,
    REMOVE_TASK,
    REMOVE_TASKS,
    REMOVE_TASK_PORT,
    GET_TASK_LIST,
    GET_TASK_PORT_LIST,
    GET_TASK_PORT_NAME,
    // Other
    HELP
};

} // namespace ocra_yarp
#endif // MESSAGE_VOCABULARY_H
