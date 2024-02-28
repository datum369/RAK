import cv2
from ultralytics import YOLO

# Load the YOLO model
model_path = 'best.pt'
model = YOLO(model_path)

# Open a connection to the webcam (change index if you have multiple cameras)
cap = cv2.VideoCapture(1)

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        break

    # Perform object detection on the frame
    results = model(frame)
    

    # Check if results is a list (no objects detected)
    if isinstance(results, list):
        continue

    cv2.imshow("result",frame)
    # Check if there are any detections
    if len(results.xyxy[0]) > 0:
        # Draw bounding boxes and labels on the frame
        for xmin, ymin, xmax, ymax, conf, cls in results.xyxy[0]:
            label = model.names[int(cls)]
            cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
            cv2.putText(frame, label, (int(xmin), int(ymin) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Result", frame)

    # Check for key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()