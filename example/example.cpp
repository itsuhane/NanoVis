#include <nanovis/nanovis.h>

class Visualization : public nanovis::NanoVis {
    double value;
public:
    Visualization() : NanoVis("NanoVis Example") {
        value = 0.5; // don't start with 0 or 1 :P

        add_button("Panel A", "Button A", [this]() {
            value = 3.772 * value * (1-value); // generate some chaos by logistic map
            notify(value); // this refreshes all widgets who are observing value
        });

        add_repeat("Panel A", "Button B", [this]() {
            value = 3.772 * value * (1-value);
            notify(value);
            return true; // returning false can force repeat to end
        });

        add_graph("Panel B", "value", value, 1.0, 0.0);
    }
};

int main() {
    Visualization vis;
    vis.show();
    return 0;
}
