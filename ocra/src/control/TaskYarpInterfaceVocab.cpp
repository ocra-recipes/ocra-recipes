#include "ocra/control/TaskYarpInterfaceVocab.h"


using namespace ocra;

const std::vector<std::string> TaskMessageHandler::TASK_MESSAGES_AS_STRINGS(
{
"OCRA_FAILURE",
"OCRA_SUCCESS",
"OCRA_WARNING",
"GET_CURRENT_STATE",
"GET_STIFFNESS",
"GET_DAMPING",
"GET_WEIGHTS",
"GET_ACTIVITY_STATUS",
"GET_DIMENSION",
"GET_TYPE",
"GET_TYPE_AS_STRING",
"GET_NAME",
"GET_CONTROL_PORT_NAMES",
"GET_TASK_PORT_NAME",
"GET_TASK_ERROR",
"GET_TASK_STATE",
"GET_DESIRED_TASK_STATE",
"GET_TASK_POSITION",
"GET_DESIRED_TASK_POSITION",
"SET_STIFFNESS",
"SET_STIFFNESS_VECTOR",
"SET_STIFFNESS_MATRIX",
"SET_DAMPING",
"SET_DAMPING_VECTOR",
"SET_DAMPING_MATRIX",
"SET_WEIGHT",
"SET_WEIGHT_VECTOR",
"SET_DESIRED_TASK_STATE",
"SET_DESIRED_TASK_POSITION",
"ACTIVATE",
"DEACTIVATE",
"TASK_IS_ACTIVATED",
"TASK_IS_DEACTIVATED",
"OPEN_CONTROL_PORTS",
"CLOSE_CONTROL_PORTS",
"HELP"
});


std::string TaskMessageHandler::taskManagerMessageTagToString(TASK_MESSAGE msg)
{
    if ((msg >= TASK_MESSAGES_AS_STRINGS.size()) || msg < 0) {
        return "";
    }
    return TASK_MESSAGES_AS_STRINGS[msg];
}

std::string TaskMessageHandler::removeUnderscores(std::string s)
{
    std::regex e ("_*");
    std::string empty("");
    return std::regex_replace (s,e,empty);
}

TASK_MESSAGE TaskMessageHandler::stringToTaskManagerMessageTag(std::string testString)
{
    // Convert to upper case
    std::transform(testString.begin(), testString.end(),testString.begin(), ::toupper);
    // Check tags "as is" i.e. with the underscores.
    for (int i=0; i<TASK_MESSAGES_AS_STRINGS.size(); ++i)
    {
        std::string s = TASK_MESSAGES_AS_STRINGS[i];
        if (testString == s) {
            return TASK_MESSAGE(i);
        }
    }
    // If the match hasn't been found then loop through again and check without underscores
    for (int i=0; i<TASK_MESSAGES_AS_STRINGS.size(); ++i)
    {
        std::string s = removeUnderscores(TASK_MESSAGES_AS_STRINGS[i]);
        if (testString == s) {
            return TASK_MESSAGE(i);
        }
    }

    return TASK_MESSAGE::OCRA_FAILURE;
}
