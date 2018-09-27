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
    class Bridge : public NanoVisWindow {
      public:
        Bridge(NanoVis *vis, const std::string &title, int width, int height) :
            NanoVisWindow(title, width, height), vis(vis), grid_visible(true) {
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

        void add_renderer(std::function<void(nanogui::GLShader &)> renderer) {
            renderers.emplace_back(renderer);
        }

        void set_grid_visible(bool visible) {
            grid_visible = visible;
        }

      protected:
        void create_world_frame() {
            world_box_points.resize(3, 114);
            world_box_colors.resize(3, 114);
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
            if (grid_visible) {
                draw_world_frame();
                draw_pickup_point();
            }
            shader.setUniform("scale", world_scale());
            for (auto &r : renderers) r(shader);
            vis->draw();
        }

        void draw_world_frame() {
            shader.uploadAttrib("point", world_box_points);
            shader.uploadAttrib("color", world_box_colors);
            shader.drawArray(GL_LINES, 0, world_box_points.cols());
        }

        void draw_pickup_point() {
            Eigen::Matrix<float, 3, 10> cursor_points;
            Eigen::Matrix<float, 3, 10> cursor_colors;
            Eigen::Vector3f p = pickup_point();
            double t = glfwGetTime();
            double cs = cos(t) * 0.1;
            double ss = sin(t) * 0.1;
            cursor_points.col(0) = Eigen::Vector3f(p.x() + cs, p.y() + ss, 0.5);
            cursor_points.col(1) = Eigen::Vector3f(p.x() - ss, p.y() + cs, 0.5);
            cursor_points.col(2) = Eigen::Vector3f(p.x() - ss, p.y() + cs, 0.5);
            cursor_points.col(3) = Eigen::Vector3f(p.x() - cs, p.y() - ss, 0.5);
            cursor_points.col(4) = Eigen::Vector3f(p.x() - cs, p.y() - ss, 0.5);
            cursor_points.col(5) = Eigen::Vector3f(p.x() + ss, p.y() - cs, 0.5);
            cursor_points.col(6) = Eigen::Vector3f(p.x() + ss, p.y() - cs, 0.5);
            cursor_points.col(7) = Eigen::Vector3f(p.x() + cs, p.y() + ss, 0.5);
            cursor_points.col(8) = Eigen::Vector3f(p.x(), p.y(), 0.0);
            cursor_points.col(9) = Eigen::Vector3f(p.x(), p.y(), 0.5);
            for (size_t i = 0; i < 10; ++i) {
                cursor_colors.col(i) = Eigen::Vector3f(0.0, 1.0, 1.0);
            }
            shader.uploadAttrib("point", cursor_points);
            shader.uploadAttrib("color", cursor_colors);
            shader.drawArray(GL_LINES, 0, cursor_points.cols());
        }

        NanoVis *vis;
        nanogui::GLShader shader;
        Eigen::Matrix<float, 3, Eigen::Dynamic> world_box_points;
        Eigen::Matrix<float, 3, Eigen::Dynamic> world_box_colors;
        std::vector<std::function<void(nanogui::GLShader &)>> renderers;
        bool grid_visible;
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

void NanoVis::add_graph(const std::string &title, const std::string &name, const double &value, const double &max_value, const double &min_value, const Eigen::Vector3d &color) {
    impl->window->add_graph(title, name, value, max_value, min_value, color);
}

void NanoVis::add_graph(const std::string &title, const std::string &name, const std::vector<double> &values, const double &max_value, const double &min_value, const Eigen::Vector3d &color) {
    impl->window->add_graph(title, name, values, max_value, min_value, color);
}

void NanoVis::add_image(const std::string &title, const std::string &name, const cv::Mat &image) {
    impl->window->add_image(title, name, image);
}

void NanoVis::add_points(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &color, const double &point_size) {
    auto renderer = [&points, color, point_size](nanogui::GLShader &shader) {
        if (points.size() > 0) {
            Eigen::MatrixXf dpoints;
            Eigen::MatrixXf dcolors;
            dpoints.resize(3, points.size());
            dcolors.resize(3, points.size());
            for (int i = 0; i < (int)points.size(); ++i) {
                dpoints.col(i) = points[i].cast<float>();
                dcolors.col(i) = color.cast<float>();
            }
            shader.uploadAttrib("point", dpoints);
            shader.uploadAttrib("color", dcolors);
            glPointSize((float)point_size);
            shader.drawArray(GL_POINTS, 0, dpoints.cols());
        }
    };
    impl->window->add_renderer(renderer);
}

void NanoVis::add_points(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &colors, const double &point_size) {
    auto renderer = [&points, &colors, point_size](nanogui::GLShader &shader) {
        if (points.size() > 0) {
            Eigen::MatrixXf dpoints;
            Eigen::MatrixXf dcolors;
            dpoints.resize(3, points.size());
            dcolors.resize(3, points.size());
            for (int i = 0; i < (int)points.size(); ++i) {
                dpoints.col(i) = points[i].cast<float>();
                dcolors.col(i) = colors[i].cast<float>();
            }
            shader.uploadAttrib("point", dpoints);
            shader.uploadAttrib("color", dcolors);
            glPointSize((float)point_size);
            shader.drawArray(GL_POINTS, 0, dpoints.cols());
        }
    };
    impl->window->add_renderer(renderer);
}

void NanoVis::add_path(const std::vector<Eigen::Vector3d> &vertices, const Eigen::Vector3d &color) {
    auto renderer = [&vertices, color](nanogui::GLShader &shader) {
        if (vertices.size() > 0) {
            Eigen::MatrixXf dpoints;
            Eigen::MatrixXf dcolors;
            dpoints.resize(3, vertices.size());
            dcolors.resize(3, vertices.size());
            for (int i = 0; i < (int)vertices.size(); ++i) {
                dpoints.col(i) = vertices[i].cast<float>();
                dcolors.col(i) = color.cast<float>();
            }
            shader.uploadAttrib("point", dpoints);
            shader.uploadAttrib("color", dcolors);
            shader.drawArray(GL_LINE_STRIP, 0, dpoints.cols());
        }
    };
    impl->window->add_renderer(renderer);
}

void NanoVis::add_path(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3d> &colors) {
    auto renderer = [&vertices, &colors](nanogui::GLShader &shader) {
        if (vertices.size() > 0) {
            Eigen::MatrixXf dpoints;
            Eigen::MatrixXf dcolors;
            dpoints.resize(3, vertices.size());
            dcolors.resize(3, vertices.size());
            for (int i = 0; i < (int)vertices.size(); ++i) {
                dpoints.col(i) = vertices[i].cast<float>();
                dcolors.col(i) = colors[i].cast<float>();
            }
            shader.uploadAttrib("point", dpoints);
            shader.uploadAttrib("color", dcolors);
            shader.drawArray(GL_LINE_STRIP, 0, dpoints.cols());
        }
    };
    impl->window->add_renderer(renderer);
}

void NanoVis::set_timeout(int refresh, const std::function<bool()> &callback) {
    impl->window->set_timeout(refresh, callback);
}

void NanoVis::set_grid_visible(bool visible) {
    impl->window->set_grid_visible(visible);
}

void NanoVis::set_camera(const Eigen::Vector3d &position, double roll, double yaw, double pitch) {
    impl->window->set_camera(position.cast<float>(), (float)roll, (float)yaw, (float)pitch);
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

void NanoVis::set_position(int x, int y) {
    impl->window->set_position(x, y);
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
