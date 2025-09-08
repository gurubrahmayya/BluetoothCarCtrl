Hello all, One of the biggest problems with toy cars is frequent battery replacement and poor control. The motors usually run at full duty cycle, making smooth left, right, or reverse movements almost impossible.

To solve this, redesigned the system. used a rechargeable Li-Ion battery with a TP4056 charging module for safe charging and protection. For control, leveraged the ESP32’s built-in Bluetooth and paired it with a DRV8833 motor driver.

On the user side, built an Android app using MIT App Inventor. The app reads accelerometer data from the phone and sends commands wirelessly through Bluetooth to the ESP32, letting you control the car just by tilting your phone.

On the technical side, applied RTOS concepts like separating cores so Core1 handles Bluetooth while Core2 manages navigation. used queues for task synchronization and scheduling to ensure smooth, lag-free operation.

In short, this project transforms a basic toy car into an interactive, smart vehicle with precise wireless control and longer battery life.

Happy Learning...
