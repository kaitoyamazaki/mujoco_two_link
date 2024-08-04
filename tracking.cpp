#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

// グローバル変数
mjModel* m = nullptr;        // MuJoCo model
mjData* d = nullptr;         // MuJoCo data
mjvCamera cam;               // カメラ
mjvOption opt;               // オプション
mjvScene scn;                // シーン
mjrContext con;              // コンテキスト

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
bool reset_simulation = false;
bool goal_reached = false;   // 目標に到達したかどうかのフラグ

// ヘルパー関数
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 初期姿勢の定義
std::pair<double, double> initial_position = {deg2rad(15), deg2rad(15)};
std::pair<double, double> goal_position;  // ゴール位置を格納するための変数

// アクチュエータの目標位置を設定する関数
void set_actuator_targets(double target1, double target2) {
    d->ctrl[mj_name2id(m, mjOBJ_ACTUATOR, "joint1")] = target1;
    d->ctrl[mj_name2id(m, mjOBJ_ACTUATOR, "joint2")] = target2;
}

// 現在のジョイント位置が目標位置に近いかどうかをチェックする関数
bool is_position_reached(double target1, double target2, double threshold = 0.01) {
    double joint1_pos = d->qpos[mj_name2id(m, mjOBJ_JOINT, "joint1")];
    double joint2_pos = d->qpos[mj_name2id(m, mjOBJ_JOINT, "joint2")];
    return (std::abs(joint1_pos - target1) < threshold) && (std::abs(joint2_pos - target2) < threshold);
}

// マウスボタンコールバック
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update last click position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// カーソル位置コールバック
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // determine action based on mouse button
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
                  glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right) {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// スクロールコールバック
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// キーコールバック
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_R) {
            // シミュレーションをリセットするフラグを立てる
            reset_simulation = true;
            goal_reached = false;  // フラグをリセット
        }
    }
}

// サイト位置を取得する関数
std::pair<double, double> get_site_position(const mjModel* m, const mjData* d, const char* site_name) {
    int site_id = mj_name2id(m, mjOBJ_SITE, site_name);
    if (site_id == -1) {
        std::cerr << "Site " << site_name << " not found" << std::endl;
        return {0.0, 0.0};  // デフォルト値を返す
    }

    const mjtNum* site_pos = d->site_xpos + 3 * site_id;
    std::cout << "Position of site " << site_name << ": "
              << site_pos[0] << ", " << site_pos[1] << ", " << site_pos[2] << std::endl;

    return {site_pos[0], site_pos[1]};
}

// 初期のサイト位置を取得する関数
std::pair<double, double> get_initial_site_position(const mjModel* m, const mjData* d, const char* site_name) {
    int site_id = mj_name2id(m, mjOBJ_SITE, site_name);
    if (site_id == -1) {
        std::cerr << "Initial site " << site_name << " not found" << std::endl;
        return {0.0, 0.0};  // デフォルト値を返す
    }

    const mjtNum* site_pos = d->site_xpos + 3 * site_id;
    std::cout << "Initial position of site " << site_name << ": "
              << site_pos[0] << ", " << site_pos[1] << ", " << site_pos[2] << std::endl;

    return {site_pos[0], site_pos[1]};
}

// 初期姿勢を設定する関数
void set_initial_position() {
    set_actuator_targets(initial_position.first, initial_position.second);
    while (!is_position_reached(initial_position.first, initial_position.second)) {
        mj_step(m, d);

        // レンダリング
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(glfwGetCurrentContext(), &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(glfwGetCurrentContext());
        glfwPollEvents();

        // 一時停止
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 初期サイト位置を取得し、goal_positionを設定
    std::pair<double, double> initial_site_pos = get_initial_site_position(m, d, "end_effector");
    goal_position = {-initial_site_pos.first, initial_site_pos.second};

    // goal_positionを出力
    std::cout << "Goal position (x, y): " << goal_position.first << ", " << goal_position.second << std::endl;
}

// ヤコビ行列の逆行列を計算する関数
void inverse_jacobian(double theta1, double theta2, double l1, double l2, double& j11, double& j12, double& j21, double& j22) {
    // ヤコビ行列 J
    double J[2][2];
    J[0][0] = -l1 * sin(theta1) - l2 * sin(theta1 + theta2);
    J[0][1] = -l2 * sin(theta1 + theta2);
    J[1][0] = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
    J[1][1] = l2 * cos(theta1 + theta2);

    // ヤコビ行列の逆行列 J^(-1)
    double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (det != 0) {
        j11 = J[1][1] / det;
        j12 = -J[0][1] / det;
        j21 = -J[1][0] / det;
        j22 = J[0][0] / det;
    } else {
        std::cerr << "Singular Jacobian matrix" << std::endl;
        j11 = j12 = j21 = j22 = 0;
    }
}

// 目標位置に向かって移動する関数
void move_to_goal() {
    double l1 = 0.1; // リンク1の長さ
    double l2 = 0.1; // リンク2の長さ
    double step_size = 0.01; // 一度に進むステップサイズ
    double threshold = 0.01; // 目標位置に近いかどうかを判定する閾値

    while (!goal_reached) {
        // 現在のジョイント角度を取得
        double theta1 = d->qpos[mj_name2id(m, mjOBJ_JOINT, "joint1")];
        double theta2 = d->qpos[mj_name2id(m, mjOBJ_JOINT, "joint2")];

        // 現在のエンドエフェクタの位置を計算
        double x_current = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
        double y_current = l1 * sin(theta1) + l2 * sin(theta1 + theta2);

        // 目標位置までの偏差を計算
        double dx = goal_position.first - x_current;
        double dy = goal_position.second - y_current;

        // 目標位置に近づいたらフラグを設定
        if (sqrt(dx*dx + dy*dy) < threshold) {
            goal_reached = true;
            break;
        }

        // 偏差をステップサイズに合わせてスケーリング
        double distance = sqrt(dx*dx + dy*dy);
        dx = (dx / distance) * step_size;
        dy = (dy / distance) * step_size;

        // 逆ヤコビ行列を計算
        double j11, j12, j21, j22;
        inverse_jacobian(theta1, theta2, l1, l2, j11, j12, j21, j22);

        // Δθを計算
        double dtheta1 = j11 * dx + j12 * dy;
        double dtheta2 = j21 * dx + j22 * dy;

        // 新しいジョイント角度を設定
        set_actuator_targets(theta1 + dtheta1, theta2 + dtheta2);
        mj_step(m, d);

        // レンダリング
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(glfwGetCurrentContext(), &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(glfwGetCurrentContext());
        glfwPollEvents();

        // 一時停止
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // サイト位置を取得
    get_site_position(m, d, "end_effector");
}

void simulate(const char* filename) {
    m = mj_loadXML(filename, nullptr, nullptr, 0);
    if (!m) {
        std::cerr << "Error loading model" << std::endl;
        return;
    }
    d = mj_makeData(m);
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        return;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
    if (!window) {
        std::cerr << "Could not create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // コールバック関数を設定
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);
    glfwSetKeyCallback(window, key_callback);

    // 最初に一度初期姿勢を設定
    set_initial_position();

    // メインループ
    while (!glfwWindowShouldClose(window)) {
        if (reset_simulation) {
            mj_resetData(m, d);
            reset_simulation = false;
            goal_reached = false;  // フラグをリセット
            set_initial_position();
        } else {
            // シミュレーションステップとレンダリング
            mj_step(m, d);
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);
            glfwSwapBuffers(window);
            glfwPollEvents();
        }

        // ゴール位置に移動
        if (!goal_reached) {
            move_to_goal();
        }
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <model file>" << std::endl;
        return 1;
    }
    simulate(argv[1]);
    return 0;
}
