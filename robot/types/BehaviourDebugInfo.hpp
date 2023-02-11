#ifndef BEHAVIOUR_DEBUG_INFO_HPP
#define BEHAVIOUR_DEBUG_INFO_HPP

class BehaviourDebugInfo
{
  public:

    // Behaviour hierarchy
    std::string bodyBehaviourHierarchy;
    std::string headBehaviourHierarchy;

    // Ball manouvering debug inormation
    bool haveBallManoeuvreTarget;  // Whether we have a ball manouvering target
    float ballManoeuvreTargetX;    // Where we are aiming to get the ball to (global X)
    float ballManoeuvreTargetY;    // Where we are aiming to get the ball to (global Y)
    float ballManoeuvreHeadingError; // How much error we are allowing (in rads)
    std::string ballManoeuvreType;   // Which manoeuvre type we're using (ie. KICK, DRIBBLE, etc.)
    bool ballManoeuvreHard;  // Whether we are manoeuvring the ball as hard as we can

    bool anticipating;  // Whether we are anticipating
    float anticipateX;  // x coordinate of anticipate pose we want to be at
    float anticipateY;  // y coordinate of anticipate pose we want to be at
    float anticipateH;  // heading of anticipate pose we want to be at

    BehaviourDebugInfo() {
        bodyBehaviourHierarchy = "";
        headBehaviourHierarchy = "";
        haveBallManoeuvreTarget = false;
        ballManoeuvreTargetX = 0.f;
        ballManoeuvreTargetY = 0.f;
        ballManoeuvreHeadingError = 0.f;
        ballManoeuvreType = "";
        ballManoeuvreHard = false;
        anticipating = false;
        anticipateX = 0.f;
        anticipateY = 0.f;
        anticipateH = 0.f;
    };
};

#endif // BEHAVIOUR_DEBUG_INFO_HPP
