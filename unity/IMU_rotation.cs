using UnityEngine;
using System.IO;
using System.IO.Ports;
using System;
using System.Linq;

public class IMU_rotation : MonoBehaviour
{
    public float x_angular_velocity, y_angular_velocity, z_angular_velocity,
                 x_radians, y_radians, z_radians;
    private Quaternion current_rotation = Quaternion.identity;
    private float lastTime = 0;


    // Serial port configuration
    private SerialPort port = new SerialPort("COM9", 115200);
    
    void Start()
    {
        port.ReadBufferSize = 8192;
        // Open the serial port
        try
        {
            port.Open();
            port.ReadTimeout = 10000; // Set timeout for reading
            Debug.Log("Serial port COM9 opened successfully.");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"Failed to open serial port COM9: {ex.Message}");
        }
    }

    void Update()
    {
        // Read data from serial port and print it
        if (port.IsOpen)
        {
            // Attempt to read data from the serial port
            string incomingData = port.ReadLine(); // Assuming the data is line-based
            Debug.Log(incomingData);
            if (incomingData.Contains("'"))
            {
                string[] values = incomingData.Split("'");
                x_angular_velocity = float.Parse(values[3]);
                y_angular_velocity = float.Parse(values[4]);
                z_angular_velocity = float.Parse(values[5]);

                float deltaTime = Time.time - lastTime;
                lastTime = Time.time;

                if (Math.Sqrt(Math.Pow(x_angular_velocity, 2) + Math.Pow(y_angular_velocity, 2) + Math.Pow(z_angular_velocity, 2)) > 0.2)
                {
                    // Process IMU rotation as before
                    Quaternion delta_rotation = Quaternion.Euler(-x_angular_velocity * Time.deltaTime,
                                                                 -z_angular_velocity * Time.deltaTime,
                                                                 -y_angular_velocity * Time.deltaTime);

                    current_rotation *= delta_rotation;
                    transform.rotation = current_rotation;
                }
            }
        }
    }

    void OnApplicationQuit()
    {
        // Close the serial port when the application quits
        if (port.IsOpen)
        {
            port.Close();
            Debug.Log("Serial port COM9 closed.");
        }
    }
}