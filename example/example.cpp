#include <nanovis/nanovis.h>

double shared_value = 0.5;

class Example : public nanovis::NanoVis {
    double value;
    std::vector<Eigen::Vector3d> points;

  public:
    Example() :
        NanoVis("NanoVis Example", 500, 500) {
        value = 0.5; // don't start with 0 or 1 :P
        points.resize(1000);
        {
            double h = 0.01;
            double a = 10.0;
            double b = 28.0;
            double c = 8.0 / 3.0;
            Eigen::Vector3d x0 = {0.1, 0.0, 0.0};
            Eigen::Vector3d x1;
            for (size_t i = 0; i < points.size(); ++i) {
                x1.x() = x0.x() + h * a * (x0.y() - x0.x());
                x1.y() = x0.y() + h * (x0.x() * (b - x0.z()) - x0.y());
                x1.z() = x0.z() + h * (x0.x() * x0.y() - c * x0.z());
                points[i] = x1 * 0.1;
                x0 = x1;
            }
        }

        add_graph("Panel B", "value", value, 1.0, 0.0);
        add_graph("Panel B", "shared value", shared_value, 1.0, 0.0);
        add_path(points);

        add_button("Panel A", "Button A", [this]() {
            value = 3.772 * value * (1 - value); // generate some chaos by logistic map
            notify(value);                       // this refreshes all widgets who are observing value
        });

        add_repeat("Panel A", "Button B", [this]() {
            value = 3.772 * value * (1 - value);
            notify(value);
            for (size_t i = 1; i < points.size(); ++i) {
                points[i - 1] = points[i];
            }
            double h = 0.01;
            double a = 10.0;
            double b = 28.0;
            double c = 8.0 / 3.0;
            const Eigen::Vector3d x0 = points.back() * 10;
            Eigen::Vector3d x1;
            x1.x() = x0.x() + h * a * (x0.y() - x0.x());
            x1.y() = x0.y() + h * (x0.x() * (b - x0.z()) - x0.y());
            x1.z() = x0.z() + h * (x0.x() * x0.y() - c * x0.z());
            points.back() = x1 * 0.1;
            notify(points);
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
