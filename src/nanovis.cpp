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
    std::unordered_set<NanoVis *> vises;

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
        for (NanoVis *other_vis : vises) {
            if (other_vis != vis) {
                other_vis->broadcast(obj, false);
            }
        }
    }

    void main(int interval) {
        nanogui::mainloop(interval);
    }
};

} // namespace nanovis

#define glsl_code(str) "#version 330\n" #str

struct NanoVis::NanoVisImpl {
    struct VcPointCloud {
        Eigen::Matrix<float, 3, Eigen::Dynamic> points;
        Eigen::Matrix<float, 3, Eigen::Dynamic> colors;
    };

    class Bridge : public NanoVisWindow {
      public:
        Bridge(NanoVis *vis, const std::string &title, int width, int height) :
            NanoVisWindow(title, width, height), vis(vis) {
            shader.init("vc_shader",
                        glsl_code(
                            uniform mat4 model_view_proj;
                            uniform float scale;
                            in vec3 point;
                            in vec3 color;
                            out vec3 c;
                            void main() {
                                c = color;
                                gl_Position = model_view_proj * vec4(scale * point, 1.0);
                            }),
                        glsl_code(
                            in vec3 c;
                            out vec4 color;
                            void main() {
                                color = vec4(c, 1.0);
                            }));
            create_world_frame();
        }

        void add_point_cloud(std::vector<Eigen::Vector3f> &points) {
            point_clouds[&points] = {};
            add_variable(points, [this, &points]() {
                VcPointCloud &vcpcl = point_clouds.at(&points);
                vcpcl.points.resize(3, points.size());
                vcpcl.colors.resize(3, points.size());
                for (size_t i = 0; i < points.size(); ++i) {
                    vcpcl.points.col(i) = points[i];
                    vcpcl.colors.col(i) = Eigen::Vector3f(1.0, 1.0, 0.0);
                }
            });
        }

      protected:
        void create_world_frame() {
            world_box_points.resize(3, 90 + 24);
            world_box_colors.resize(3, 90 + 24);
            for (int i = -10; i <= 10; ++i) {
                if (i != 0) {
                    world_box_points.col((i + 10) * 4 + 0) = Eigen::Vector3f(i, -10, 0);
                    world_box_points.col((i + 10) * 4 + 1) = Eigen::Vector3f(i, +10, 0);
                    world_box_points.col((i + 10) * 4 + 2) = Eigen::Vector3f(-10, i, 0);
                    world_box_points.col((i + 10) * 4 + 3) = Eigen::Vector3f(+10, i, 0);
                } else {
                    world_box_points.col((i + 10) * 4 + 0) = Eigen::Vector3f(i, -10, 0);
                    world_box_points.col((i + 10) * 4 + 1) = Eigen::Vector3f(i, 0, 0);
                    world_box_points.col((i + 10) * 4 + 2) = Eigen::Vector3f(-10, i, 0);
                    world_box_points.col((i + 10) * 4 + 3) = Eigen::Vector3f(0, i, 0);
                }
                if (i != 0 && i != -10 && i != 10) {
                    world_box_colors.col((i + 10) * 4 + 0) = Eigen::Vector3f(0.4, 0.4, 0.4);
                    world_box_colors.col((i + 10) * 4 + 1) = Eigen::Vector3f(0.4, 0.4, 0.4);
                    world_box_colors.col((i + 10) * 4 + 2) = Eigen::Vector3f(0.4, 0.4, 0.4);
                    world_box_colors.col((i + 10) * 4 + 3) = Eigen::Vector3f(0.4, 0.4, 0.4);
                } else {
                    world_box_colors.col((i + 10) * 4 + 0) = Eigen::Vector3f(0.55, 0.55, 0.55);
                    world_box_colors.col((i + 10) * 4 + 1) = Eigen::Vector3f(0.55, 0.55, 0.55);
                    world_box_colors.col((i + 10) * 4 + 2) = Eigen::Vector3f(0.55, 0.55, 0.55);
                    world_box_colors.col((i + 10) * 4 + 3) = Eigen::Vector3f(0.55, 0.55, 0.55);
                }
            }
            for (int i = 0; i < 3; ++i) {
                world_box_points.col(84 + i * 2 + 0) = Eigen::Vector3f(0, 0, 0);
                world_box_points.col(84 + i * 2 + 1) = Eigen::Vector3f::Unit(i) * 10;
                world_box_colors.col(84 + i * 2 + 0) = Eigen::Vector3f::Unit(i) * 0.75;
                world_box_colors.col(84 + i * 2 + 1) = Eigen::Vector3f::Unit(i) * 0.75;
            }
            world_box_points.col(90) = Eigen::Vector3f(-10, -10, -10);
            world_box_points.col(91) = Eigen::Vector3f(-10, -10, +10);
            world_box_points.col(92) = Eigen::Vector3f(-10, -10, -10);
            world_box_points.col(93) = Eigen::Vector3f(-10, +10, -10);
            world_box_points.col(94) = Eigen::Vector3f(-10, -10, -10);
            world_box_points.col(95) = Eigen::Vector3f(+10, -10, -10);
            world_box_points.col(96) = Eigen::Vector3f(+10, +10, -10);
            world_box_points.col(97) = Eigen::Vector3f(+10, -10, -10);
            world_box_points.col(98) = Eigen::Vector3f(+10, +10, -10);
            world_box_points.col(99) = Eigen::Vector3f(-10, +10, -10);
            world_box_points.col(100) = Eigen::Vector3f(+10, +10, -10);
            world_box_points.col(101) = Eigen::Vector3f(+10, +10, +10);
            world_box_points.col(102) = Eigen::Vector3f(+10, -10, +10);
            world_box_points.col(103) = Eigen::Vector3f(+10, +10, +10);
            world_box_points.col(104) = Eigen::Vector3f(+10, -10, +10);
            world_box_points.col(105) = Eigen::Vector3f(-10, -10, +10);
            world_box_points.col(106) = Eigen::Vector3f(+10, -10, +10);
            world_box_points.col(107) = Eigen::Vector3f(+10, -10, -10);
            world_box_points.col(108) = Eigen::Vector3f(-10, +10, +10);
            world_box_points.col(109) = Eigen::Vector3f(+10, +10, +10);
            world_box_points.col(110) = Eigen::Vector3f(-10, +10, +10);
            world_box_points.col(111) = Eigen::Vector3f(-10, -10, +10);
            world_box_points.col(112) = Eigen::Vector3f(-10, +10, +10);
            world_box_points.col(113) = Eigen::Vector3f(-10, +10, -10);
            for (size_t i = 0; i < 24; ++i) {
                world_box_colors.col(90 + i) = Eigen::Vector3f(0.4, 0.4, 0.4);
            }
        }

        void draw() override {
            Eigen::Matrix4f mvpmat = proj_matrix(0.01, 1000) * view_matrix() * world_matrix();
            shader.bind();
            shader.setUniform("model_view_proj", mvpmat);
            shader.setUniform("scale", 1.0);
            glEnable(GL_DEPTH_TEST);
            draw_world_frame();
            shader.setUniform("scale", world_scale());
            draw_point_clouds();
            vis->draw();
        }

        void draw_world_frame() {
            shader.uploadAttrib("point", world_box_points);
            shader.uploadAttrib("color", world_box_colors);
            shader.drawArray(GL_LINES, 0, world_box_points.cols());
        }

        void draw_point_clouds() {
            glPointSize(5.0);
            for (const auto &vcpcl : point_clouds) {
                if (vcpcl.second.points.cols() > 0) {
                    shader.uploadAttrib("point", vcpcl.second.points);
                    shader.uploadAttrib("color", vcpcl.second.colors);
                    shader.drawArray(GL_POINTS, 0, vcpcl.second.points.cols());
                }
            }
        }

        NanoVis *vis;
        nanogui::GLShader shader;
        Eigen::Matrix<float, 3, Eigen::Dynamic> world_box_points;
        Eigen::Matrix<float, 3, Eigen::Dynamic> world_box_colors;
        std::unordered_map<std::vector<Eigen::Vector3f> *, VcPointCloud> point_clouds;
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

void NanoVis::add_point_cloud(std::vector<Eigen::Vector3f> &points) {
    impl->window->add_point_cloud(points);
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
    if (global) {
        NanoVisSystem::system().broadcast(this, value);
    }
}

void nanovis::main(int interval) {
    NanoVisSystem::system().main(interval);
}
