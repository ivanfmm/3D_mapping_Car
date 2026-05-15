import cv2

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

if ret:
    h, w = frame.shape[:2]
    left  = frame[:, :w//2]    # mitad izquierda
    right = frame[:, w//2:]    # mitad derecha
    
    cv2.imwrite('lef2.jpg', left)
    cv2.imwrite('righ2.jpg', right)
    print(f"Frame guardado: {w}x{h}")
    print(f"Cada ojo: {w//2}x{h}")

cap.release()
