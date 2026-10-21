// Placeholder lamp Action that only logs input; replaced by the Hue action in Task 9.
#include "../core/action.h"
#include "../core/net.h"

class NullAction : public Action {
   public:
    void begin(uint32_t) override {}
    void onClicks(int clicks) override { netLog("clicks %d", clicks); }
    void onPress() override { netLog("press"); }
    void loop(uint32_t) override {}
    uint32_t firstAckMs() const override { return 0; }
    void end() override {}
};

Action &makeAction() {
    static NullAction action;
    return action;
}
