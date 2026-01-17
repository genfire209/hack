import cv2
import mediapipe as mp
import numpy as np
from math import hypot
import pyautogui
import subprocess
import os

# Disable pyautogui failsafe for smoother control
pyautogui.FAILSAFE = False

# Volume control setup for Linux
volume = None
min_vol = 0.0
max_vol = 1.0
volume_available = False

try:
    result = subprocess.run(['wpctl', 'status'],capture_output=True, text=True, timeout=2)
    if result.returncode == 0:
        volume_available = True
        print(f"✓ Volume control initialized successfully!")
        print(f"  Using PipeWire (wpctl)")
    else:
        volume_available = False
except Exception as e:
    print(f"✗ Volume control initialization failed: {e}")
    print("  Trying alternative volume control methods...")
    volume_available = False

# Brightness control setup for Linux
brightness_available = False
brightness_path = None

# Try to find backlight device
try:
    backlight_dir = "/sys/class/backlight/"
    if os.path.exists(backlight_dir):
        devices = os.listdir(backlight_dir)
        if devices:
            brightness_path = os.path.join(backlight_dir, devices[0])
            # Read max brightness
            with open(os.path.join(brightness_path, "max_brightness"), 'r') as f:
                max_brightness = int(f.read().strip())
            brightness_available = True
            print(f"✓ Brightness control initialized!")
            print(f"  Using device: {devices[0]}")
except Exception as e:
    print(f"✗ Brightness control failed: {e}")
    print("  Brightness control will be disabled!")

def set_volume_linux(volume_percent):
    """Set system volume using PipeWire (wpctl)"""
    try:
        # wpctl expects volume as decimal (0.0 to 1.0+)
        volume_level = volume_percent / 100.0
        subprocess.run(['wpctl', 'set-volume', '@DEFAULT_AUDIO_SINK@', str(volume_level)], 
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"Volume control error: {e}")

def set_brightness_linux(brightness_percent):
    """Set screen brightness using sysfs"""
    try:
        if brightness_available:
            brightness_value = int((brightness_percent / 100.0) * max_brightness)
            # Use pkexec or sudo to write to brightness file
            with open(os.path.join(brightness_path, 'brightness'), 'w') as f:
                f.write(str(brightness_value))
        else:
            # Fallback to xrandr
            subprocess.run(['xrandr', '--output', 'eDP-1', '--brightness', 
                          str(brightness_percent / 100.0)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except:
        pass

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Webcam setup
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not access the webcam.")
    exit()

# Get screen dimensions for cursor control
screen_w, screen_h = pyautogui.size()

# Variables for gesture control
prev_x, prev_y = 0, 0
click_threshold = 30  # Distance threshold for click detection
scroll_cooldown = 0
mode = "volume"  # Modes: "volume", "brightness", "cursor", "scroll"

print("\n" + "="*60)
print("ADVANCED GESTURE CONTROL FOR ARCH LINUX!")
print("="*60)
print("GESTURE CONTROLS:")
print("  👉 RIGHT hand - Thumb & Index pinch:")
print("     • Normal: Control VOLUME")
print("  👉 LEFT hand - Thumb & Index pinch:")
print("     • Normal: Control BRIGHTNESS")
print("  👉 RIGHT hand - Index finger up (peace sign):")
print("     • Move cursor, pinch to LEFT CLICK")
print("  👉 LEFT hand - FIST (all fingers closed):")
print("     • Scroll UP/DOWN based on fist height")
print("  👉 Press 'Q' to Quit")
print("="*60 + "\n")

def is_fist(hand_landmarks):
    """Detect if hand is in a fist position"""
    # Get fingertip and base positions
    fingers = []
    tips = [mp_hands.HandLandmark.INDEX_FINGER_TIP, 
            mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            mp_hands.HandLandmark.RING_FINGER_TIP, 
            mp_hands.HandLandmark.PINKY_TIP]
    
    mcp = [mp_hands.HandLandmark.INDEX_FINGER_MCP,
           mp_hands.HandLandmark.MIDDLE_FINGER_MCP,
           mp_hands.HandLandmark.RING_FINGER_MCP,
           mp_hands.HandLandmark.PINKY_MCP]
    
    # Check if fingertips are below MCPs (finger folded)
    for tip, base in zip(tips, mcp):
        if hand_landmarks.landmark[tip].y > hand_landmarks.landmark[base].y:
            fingers.append(1)
        else:
            fingers.append(0)
    
    # Fist if all 4 fingers are folded
    return sum(fingers) >= 3

def is_peace_sign(hand_landmarks):
    """Detect peace sign (index and middle finger up)"""
    # Index finger up
    index_up = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y < \
               hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y
    
    # Middle finger up
    middle_up = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < \
                hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
    
    # Ring and pinky down
    ring_down = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y > \
                hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y
    
    pinky_down = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y > \
                 hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y
    
    return index_up and middle_up and ring_down and pinky_down

while True:
    success, img = cap.read()
    if not success:
        print("Failed to read frame from webcam.")
        break

    img = cv2.flip(img, 1)  # Flip the image for a mirror effect
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    
    h, w, _ = img.shape

    if results.multi_hand_landmarks and results.multi_handedness:
        for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
            hand_label = results.multi_handedness[i].classification[0].label  # 'Left' or 'Right'

            # Draw hand landmarks
            mp_draw.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Extract finger positions
            thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
            index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]

            thumb_pos = (int(thumb_tip.x * w), int(thumb_tip.y * h))
            index_pos = (int(index_tip.x * w), int(index_tip.y * h))

            # Calculate distances
            distance = hypot(index_pos[0] - thumb_pos[0], index_pos[1] - thumb_pos[1])

            # RIGHT HAND GESTURES
            if hand_label == "Right":
                # Check for peace sign (cursor control mode)
                if is_peace_sign(hand_landmarks):
                    # Draw cursor control indicator
                    cv2.putText(img, 'CURSOR MODE', (w - 200, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    
                    # Use index finger to control cursor
                    index_x = int(index_tip.x * screen_w)
                    index_y = int(index_tip.y * screen_h)
                    
                    # Smooth cursor movement
                    smooth_x = prev_x + (index_x - prev_x) // 3
                    smooth_y = prev_y + (index_y - prev_y) // 3
                    
                    try:
                        print(f"Moving cursor to {smooth_x}, {smooth_y}")  # Debug
                        pyautogui.moveTo(smooth_x, smooth_y, duration=0)
                    except Exception as e:
                        print(f"Cursor move failed: {e}")
                    prev_x, prev_y = smooth_x, smooth_y
                    
                    # Draw cursor indicator
                    cv2.circle(img, index_pos, 15, (0, 255, 255), cv2.FILLED)
                    
                    # Check for pinch (left click)
                    if distance < click_threshold:
                        pyautogui.click()
                        cv2.putText(img, 'CLICK!', (w - 200, 90),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                # Volume control (thumb & index pinch)
                else:
                    vol_percent = int(np.interp(distance, [30, 150], [0, 100]))
                    set_volume_linux(vol_percent)
                    
                    # Visual feedback for volume
                    vol_bar = np.interp(distance, [30, 150], [400, 150])
                    
                    cv2.rectangle(img, (50, 150), (85, 400), (255, 0, 0), 2)
                    cv2.rectangle(img, (50, int(vol_bar)), (85, 400), (255, 0, 0), cv2.FILLED)
                    cv2.putText(img, f'Volume: {vol_percent}%', (40, 120),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    
                    # Draw circles at the tips
                    cv2.circle(img, thumb_pos, 10, (255, 0, 0), cv2.FILLED)
                    cv2.circle(img, index_pos, 10, (255, 0, 0), cv2.FILLED)
                    cv2.line(img, thumb_pos, index_pos, (0, 255, 0), 3)

            # LEFT HAND GESTURES
            elif hand_label == "Left":
                # Check for fist (scroll mode)
                if is_fist(hand_landmarks):
                    cv2.putText(img, 'SCROLL MODE', (50, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                    
                    # Get wrist position for scroll control
                    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
                    wrist_y = wrist.y
                    
                    # Scroll based on fist position (top = scroll up, bottom = scroll down)
                    if scroll_cooldown == 0:
                        if wrist_y < 0.4:  # Upper part of screen
                            pyautogui.scroll(30)  # Scroll up
                            cv2.putText(img, 'SCROLL UP', (50, 90),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                            scroll_cooldown = 10
                        elif wrist_y > 0.6:  # Lower part of screen
                            pyautogui.scroll(-30)  # Scroll down
                            cv2.putText(img, 'SCROLL DOWN', (50, 90),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                            scroll_cooldown = 10
                    
                    # Draw fist indicator
                    wrist_pos = (int(wrist.x * w), int(wrist.y * h))
                    cv2.circle(img, wrist_pos, 20, (255, 255, 0), cv2.FILLED)
                
                # Brightness control (thumb & index pinch)
                else:
                    brightness = int(np.interp(distance, [30, 150], [0, 100]))
                    set_brightness_linux(brightness)
                    
                    # Visual feedback for brightness
                    brightness_bar = np.interp(distance, [30, 150], [400, 150])
                    cv2.rectangle(img, (100, 150), (135, 400), (0, 255, 0), 2)
                    cv2.rectangle(img, (100, int(brightness_bar)), (135, 400), (0, 255, 0), cv2.FILLED)
                    cv2.putText(img, f'Brightness: {brightness}%', (90, 450),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Draw circles at the tips
                    cv2.circle(img, thumb_pos, 10, (0, 255, 0), cv2.FILLED)
                    cv2.circle(img, index_pos, 10, (0, 255, 0), cv2.FILLED)
                    cv2.line(img, thumb_pos, index_pos, (0, 255, 0), 3)
    
    # Decrease scroll cooldown
    if scroll_cooldown > 0:
        scroll_cooldown -= 1

    # Show the video feed with annotations
    cv2.imshow("Advanced Gesture Control - Arch Linux", img)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()
hands.close()
print("\nGesture control stopped. Thanks for using!")
