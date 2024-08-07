#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

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

// ヘルパー関数
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// ジョイント位置のシーケンスを定義
std::vector<std::pair<double, double>> sequence = {
    {0, 0},
    {0, deg2rad(90)},
    {deg2rad(45), deg2rad(90)},
    {deg2rad(90), deg2rad(-90)},
    {deg2rad(45), deg2rad(-135)},
    {0, 0}  // 初期姿勢に戻る
};

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
        }
    }
}

// シーケンスの実行関数
void run_sequence() {
    for (const auto& pos : sequence) {
        set_actuator_targets(pos.first, pos.second);
        int steps = 0;
        while (!is_position_reached(pos.first, pos.second) && steps < 1000) {  // 最大ステップ数を設定して無限ループを防止
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
            steps++;
        }

        // リセットが要求された場合、シーケンスを中断
        if (reset_simulation) {
            break;
        }
    }
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

    // 最初に一度シーケンスを実行
    run_sequence();

    // メインループ
    while (!glfwWindowShouldClose(window)) {
        if (reset_simulation) {
            mj_resetData(m, d);
            reset_simulation = false;
            run_sequence();
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
