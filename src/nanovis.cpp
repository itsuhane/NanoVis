#include <nanovis/nanovis.h>
#include <unordered_set>
#include "window.h"

using namespace nanovis;

namespace nanovis {

class NanoVisSystem {
    NanoVisSystem() {
        nanogui::init();
    }
    ~NanoVisSystem() {
        nanogui::shutdown();
    }
    std::unordered_set<NanoVis*> vises;
public:
    static NanoVisSystem &system() {
        static NanoVisSystem sys;
        return sys;
    }

    void register_vis(NanoVis *vis) {
        vises.insert(vis);
    }

    void unregister_vis(NanoVis *vis) {
        vises.erase(vis);
    }

    void broadcast(NanoVis *vis, const void *obj) {
        for(NanoVis *other_vis : vises) {
            if(other_vis!=vis) {
                other_vis->broadcast(obj, false);
            }
        }
    }

    void main(int interval) {
        nanogui::mainloop(interval);
    }
};

}

struct NanoVis::NanoVisImpl {
    class Bridge : public NanoVisWindow {
    public:
        Bridge(NanoVis *vis, const std::string &title, int width, int height) : NanoVisWindow(title, width, height), vis(vis) {}
    protected:
        void draw() override {
            vis->draw();
        }

        NanoVis *vis;
    };
    std::unique_ptr<Bridge> window;
};

NanoVis::NanoVis(const std::string &title, int width, int height) {
    NanoVisSystem::system().register_vis(this);
    impl = std::make_unique<NanoVisImpl>();
    impl->window = std::make_unique<NanoVisImpl::Bridge>(this, title, width, height);
}

NanoVis::~NanoVis() {
    NanoVisSystem::system().unregister_vis(this);
}

void NanoVis::show() {
    impl->window->show();
}

void NanoVis::refresh() {
    impl->window->refresh();
}

int NanoVis::width() const {
    return impl->window->width();
}

int NanoVis::height() const {
    return impl->window->height();
}

void NanoVis::add_button(const std::string &title, const std::string &name, const std::function<void()> &callback) {
    impl->window->add_button(title, name, callback);
}

void NanoVis::add_toggle(const std::string &title, const std::string &name, const std::function<void(bool)> &callback) {
    impl->window->add_toggle(title, name, callback);
}

void NanoVis::add_toggle(const std::string &title, const std::string &name, bool &value, const std::function<void(bool)> &callback) {
    impl->window->add_toggle(title, name, value, callback);
}

void NanoVis::add_repeat(const std::string &title, const std::string &name, const std::function<bool()> &callback) {
    impl->window->add_repeat(title, name, callback);
}

void NanoVis::add_graph(const std::string &title, const std::string &name, double &value, const double &max_value, const double &min_value) {
    impl->window->add_graph(title, name, value, max_value, min_value);
}

void NanoVis::add_graph(const std::string &title, const std::string &name, std::vector<double> &values, const double &max_value, const double &min_value) {
    impl->window->add_graph(title, name, values, max_value, min_value);
}

void NanoVis::add_image(const std::string &title, const std::string &name, cv::Mat &image) {
    impl->window->add_image(title, name, image);
}

Eigen::Matrix4f NanoVis::proj_matrix(float near, float far) const {
    return impl->window->proj_matrix(near, far);
}

Eigen::Matrix4f NanoVis::view_matrix() const {
    return impl->window->view_matrix();
}

Eigen::Matrix4f NanoVis::world_matrix() const {
    return impl->window->world_matrix();
}

Eigen::Matrix4f NanoVis::model_view_proj() const {
    return proj_matrix() * view_matrix() * world_matrix();
}

float NanoVis::world_scale() const {
    return impl->window->world_scale();
}

void NanoVis::draw() {
}

void NanoVis::broadcast(const void *value, bool global) {
    impl->window->broadcast(value);
    if(global) {
        NanoVisSystem::system().broadcast(this, value);
    }
}

void nanovis::main(int interval) {
    NanoVisSystem::system().main(interval);
}
