import cv2

# Cambia el índice (0, 1, 2...) según corresponda a tu cámara externa
cap = cv2.VideoCapture(1)

# Intentar forzar la resolución
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Presiona la tecla 'q' para cerrar la ventana.")

while True:
    # Capturar frame por frame
    ret, frame = cap.read()
    
    if not ret:
        print("No se pudo recibir el frame. Saliendo...")
        break

    # Muestra el frame en una ventana llamada "Mi Camara"
    cv2.imshow("Mi Camara", frame)

    # cv2.waitKey(1) espera 1 milisegundo antes de avanzar.
    # Si detecta que presionaste la tecla 'q', rompe el bucle.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Al terminar, liberamos la cámara y cerramos las ventanas abiertas
cap.release()
cv2.destroyAllWindows()