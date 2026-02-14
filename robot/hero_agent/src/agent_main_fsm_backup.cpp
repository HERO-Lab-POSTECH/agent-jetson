// ==============================
// FSM Code Backup (removed from agent_main.cpp)
// Original autonomous control process: Init → CtrlEnable → Search → Detected → Descend → Grip → Lift
// ==============================

// FSM control process variables
int control_process = 0;
int process_count = 0;
int gripper_state = 0;
float start_target_depth = 0.2f;

// Object detection (vision) - used by FSM for auto-detect
const int FRAME_RATE = 10;
const int BUFFER_SIZE = FRAME_RATE * 2;
std::deque<bool> detection_buffer;
int detection_count = 0;
int object_detected = 0;
int object_centered = 0;

// Image callback for object detection
void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image;

        if (input_image.empty()) return;

        const double MIN_CONTOUR_AREA = 500.0;
        int image_center_x = input_image.cols / 2;
        int image_center_y = input_image.rows / 2;

        cv::Mat hsv, sat_mask;
        cv::cvtColor(input_image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 50, 0), cv::Scalar(255, 200, 255), sat_mask);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(sat_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool frame_detected = false;
        bool is_centered = false;
        std::vector<cv::Point> largest_contour;
        double max_area = 0.0;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > MIN_CONTOUR_AREA && area > max_area) {
                max_area = area;
                largest_contour = contour;
                frame_detected = true;
            }
        }

        if (frame_detected && !largest_contour.empty()) {
            cv::Rect bbox = cv::boundingRect(largest_contour);
            int center_x = bbox.x + bbox.width / 2;
            int center_y = bbox.y + bbox.height / 2;
            is_centered = (
                std::abs(center_x - image_center_x) < image_center_x * 0.2 &&
                std::abs(center_y - image_center_y) < image_center_y * 0.2
            );
        }

        detection_buffer.push_back(frame_detected);
        if (frame_detected) detection_count++;
        if (detection_buffer.size() > BUFFER_SIZE) {
            if (detection_buffer.front()) detection_count--;
            detection_buffer.pop_front();
        }

        object_detected = (detection_buffer.size() == BUFFER_SIZE &&
            static_cast<float>(detection_count) / BUFFER_SIZE >= 0.9f) ? 1 : 0;
        object_centered = (object_detected && is_centered) ? 1 : 0;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// FSM state machine (inside msg_callback_state)
// States: 0=Init, 1=CtrlEnable, 2=Search, 3=Detected, 4=Descend, 5=Grip, 6=Lift
void fsm_run(/* called from msg_callback_state after updating state variables */)
{
    const float DEPTH_TOL = 0.01;

    switch (control_process)
    {
        case 0: // Init
            start_target_depth = 0.1;
            process_count = 0;
            gripper_state = 0;
            send_command('h');
            send_command(';');
            send_command('g');
            cont_ver_msg.data = 3;
            pub_cont_ver.publish(cont_ver_msg);
            break;

        case 1: // CtrlEnable
            cont_ver_msg.data = cont_ver;
            pub_cont_ver.publish(cont_ver_msg);
            send_command('c');
            process_count++;
            if (!control_yaw_enabled) send_command('y');
            if (!control_depth_enabled) send_command('p');
            if (process_count % 100 == 0) adjust_depth_if_needed();
            break;

        case 2: // Search
            process_count++;
            if (move_speed < 10) send_command('z');
            else if (move_speed > 10) send_command('x');
            if (process_count % 100 == 0) adjust_depth_if_needed();
            if (control_state != 2) send_command('w');
            if (object_centered == 1) { process_count = 0; control_process = 3; }
            break;

        case 3: // Detected
            if (control_state != 5) send_command('1');
            process_count++;
            if (process_count > 200) { process_count = 0; control_process = 4; }
            break;

        case 4: // Descend
            process_count++;
            if (process_count > 20) {
                process_count = 0;
                send_command('o');
                if (gripping_depth < depth) {
                    control_process = 5; gripper_state = 0; process_count = 0;
                }
            }
            break;

        case 5: // Grip
            if (gripper_state == 0) send_command('b');
            process_count++;
            if (process_count > 100 && gripper_state < 2) { gripper_state++; process_count = 0; }
            if (gripper_state > 1) { process_count = 0; control_process = 6; }
            break;

        case 6: // Lift
            process_count++;
            if (process_count % 100 == 0) adjust_depth_if_needed();
            if (target_depth <= start_target_depth + DEPTH_TOL) {
                gripper_state = 0; process_count = 0; send_command('1');
            }
            break;
    }
}

// adjust_depth_if_needed (used by FSM)
void adjust_depth_if_needed()
{
    float delta = target_depth - start_target_depth;
    if (delta > DEPTH_TOLERANCE) send_command('l');
    else if (delta < -DEPTH_TOLERANCE) send_command('o');
}

// gripping_depth_callback (used by FSM)
void gripping_depth_callback(const std_msgs::Float64& depth_msg)
{
    gripping_depth = depth_msg.data;
}

// key_input_callback FSM control keys (0, -, =)
// In FSM mode: '0' reset, '-' prev state, '=' next state
