import os
import cv2
import picamera
import picamera.array
import numpy as np

file_dir = os.path.dirname(os.path.realpath(__file__)) + '/haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(file_dir)

try:
    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.resolution = (640,480)
            while True:
                camera.capture(stream, 'bgr', use_video_port=True)
                frame = stream.array
                frame_center = [camera.resolution[0]//2, camera.resolution[0]//2]

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
                face_vector_norm = []

                if len(faces) > 0:
                    idx_max_face = np.argmax(faces[:,2]*faces[:,3])
                    (x, y, w, h) = faces[idx_max_face,:]
                    face_center = [x + w // 2, y + h // 2]
                    face_vector = [face_center[0] - frame_center[0], face_center[1] - frame_center[1]]
                    face_vector_norm = [face_vector[0]/max(frame.shape)*2,face_vector[1]/max(frame.shape)*2]
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                print(face_vector_norm)

                cv2.imshow('frame',frame)

                # press 'q' to quit
                if cv2.waitKey(1) == ord('q'):
                    break

                # Reset the stream before the next capture
                stream.seek(0)
                stream.truncate()
    cv2.destroyAllWindows()
except KeyboardInterrupt:
    cv2.destroyAllWindows()