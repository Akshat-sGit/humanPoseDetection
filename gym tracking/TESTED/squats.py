#TEST COMPLETE

import cv2
import mediapipe as mp
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

def calculate_angle(a, b, c):
    a = np.array(a)  # First
    b = np.array(b)  # Mid
    c = np.array(c)  # End

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle

    return angle

#squats
camera = cv2.VideoCapture(0)
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

counter = 0
stage = None

# Define codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('squats.mp4', fourcc, 20.0, (640, 480))

while True:
    ret, frame = camera.read()
    if not ret:
        break

    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image.flags.writeable = False

    results = pose.process(image)

    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    try:
        landmarks = results.pose_landmarks.landmark
        left_hip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x, landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
        right_hip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
        left_knee = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y]
        right_knee = [landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].y]

        angle_left = calculate_angle(left_hip, left_knee, [left_knee[0], left_hip[1]])
        angle_right = calculate_angle(right_hip, right_knee, [right_knee[0], right_hip[1]])

        cv2.putText(image, "Left Angle: " + str(angle_left), (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(image, "Right Angle: " + str(angle_right), (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

        if angle_left > 150 and angle_right > 150:
            stage = "up"
        if angle_left < 90 and angle_right < 90 and stage == "up":
            stage = "down"
            counter += 1
            print("Repetition Count:", counter)
    except Exception as e:
        print("Error:", e)

    cv2.rectangle(image, (0, 0), (225, 73), (245, 117, 16), -1)

    cv2.putText(image, 'REPS', (15, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(image, str(counter), (80, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.putText(image, 'STAGE', (15, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(image, str(stage), (80, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

    mp.solutions.drawing_utils.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                              landmark_drawing_spec = mp.solutions.drawing_utils.DrawingSpec(color = (255, 255, 255),
                                                                                                           thickness = 2,
                                                                                                           circle_radius = 2))

    mp.solutions.drawing_utils.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                              connection_drawing_spec = mp.solutions.drawing_utils.DrawingSpec(color = (0, 0, 0),
                                                                                                             thickness = 2,
                                                                                                             circle_radius = 2))

    cv2.imshow('frame', image)
    out.write(image)  # Write frame to video

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
out.release()  # Release the video writer
cv2.destroyAllWindows()