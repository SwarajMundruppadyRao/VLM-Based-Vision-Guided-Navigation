# DetectedText.msg - Message for OCR detected text with metadata

std_msgs/Header header
string text                  # Detected text content
float32 confidence          # Confidence score (0-100)
geometry_msgs/Point position # Position in image coordinates
