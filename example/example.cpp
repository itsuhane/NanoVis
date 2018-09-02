#include <nanovis/nanovis.h>

double shared_value = 0.5;

class Example : public nanovis::NanoVis {
    double value;
    std::vector<Eigen::Vector3f> points;

  public:
    Example() :
        NanoVis("NanoVis Example", 500, 500) {
        value = 0.5; // don't start with 0 or 1 :P
        points.resize(100);
        for (auto& p : points) {
            p.setRandom();
        }

        add_graph("Panel B", "value", value, 1.0, 0.0);
        add_graph("Panel B", "shared value", shared_value, 1.0, 0.0);
        add_point_cloud(points);

        add_button("Panel A", "Button A", [this]() {
            value = 3.772 * value * (1 - value); // generate some chaos by logistic map
            notify(value);                       // this refreshes all widgets who are observing value
            for (auto& p : points) {
                p.setRandom();
            }
            notify(points);
        });

        add_repeat("Panel A", "Button B", [this]() {
            value = 3.772 * value * (1 - value);
            notify(value);
            return true; // returning false can force repeat to end
        });

        static bool is_first_window = true;
        if (is_first_window) {
            add_repeat("Panel A", "Button C", [this]() {
                shared_value = 3.772 * shared_value * (1 - shared_value);
                notify(shared_value);
                return true; // returning false can force repeat to end
            });
            is_first_window = false;
            set_position(15, 60);
        } else {
            set_position(530, 60);
        }
    }
};

int main() {
    Example vis1, vis2;
    vis1.show();
    vis2.show();
    nanovis::main();
    return 0;
}
