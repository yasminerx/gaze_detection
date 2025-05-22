import cv2
import mediapipe as mp
import numpy as np

def test_mediapipe_face_detection():
    mp_face_mesh = mp.solutions.face_mesh
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    # static image mode :
    # false : process video stream -> less latency
    # truc : face detection run on all images (good for unrelated images)
    face_mesh = mp_face_mesh.FaceMesh(static_image_mode=True,
                                      refine_landmarks=True,
                                      max_num_faces=2)
    print("ok")
    image = cv2.imread("test_image.jpg")
    if image is None:
        raise ValueError("error loading the image")
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(image_rgb)
    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            print("face landmarks detected")
            # image = draw_full_face_mesh(image, face_landmarks)
            image = draw_pupil(image, face_landmarks)
            
            cv2.imshow("face mesh", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    else:
        print("no face detected")
    face_mesh.close()

    
def draw_full_face_mesh(image, face_landmarks):
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_face_mesh = mp.solutions.face_mesh
    mp_drawing.draw_landmarks(
        image=image,  # image in BGR (OpenCV)
        landmark_list=face_landmarks,
        connections=mp_face_mesh.FACEMESH_TESSELATION,
        landmark_drawing_spec=None,
        connection_drawing_spec=mp_drawing.DrawingSpec(color=(0,255,0), thickness=1, circle_radius=1)
    )
    mp_drawing.draw_landmarks(
        image=image,
        landmark_list=face_landmarks,
        connections=mp_face_mesh.FACEMESH_CONTOURS,
        landmark_drawing_spec=None,
        connection_drawing_spec=mp_drawing_styles
        .get_default_face_mesh_contours_style())
    mp_drawing.draw_landmarks(
        image=image,
        landmark_list=face_landmarks,
        connections=mp_face_mesh.FACEMESH_IRISES,
        landmark_drawing_spec=None,
        connection_drawing_spec=mp_drawing_styles
        .get_default_face_mesh_iris_connections_style())
    return image

def draw_pupil(image, face_landmarks):
    mp_drawing = mp.solutions.drawing_utils
    mp_face_mesh = mp.solutions.face_mesh
    # Draw the eyes
    lst = [468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 33, 133, 362, 263]
    for i in lst:
        landmark = face_landmarks.landmark[i]
        x = int(landmark.x * image.shape[1])
        y = int(landmark.y * image.shape[0])
        cv2.circle(image, (x, y), 1, (0, 255, 0), -1)

    ## pour avoir le mesh de contour des yeux :
    # mp.solutions.drawing_utils.draw_landmarks(
    #     image, face_landmarks,
    #     mp.solutions.face_mesh.FACEMESH_LEFT_EYE,
    #     landmark_drawing_spec = None
    # )
    # mp.solutions.drawing_utils.draw_landmarks(
    #     image, face_landmarks,
    #     mp.solutions.face_mesh.FACEMESH_RIGHT_EYE,
    #     landmark_drawing_spec = None
    # )
    return image

test_mediapipe_face_detection()
