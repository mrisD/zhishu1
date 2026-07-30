using System;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;

public sealed class QuestControllerTcpSender : MonoBehaviour
{
    private const uint Magic = 0x31525651; // "QVR1" little-endian
    private const uint ProtocolVersion = 1;

    [Header("Robot computer connection")]
    [Tooltip("Use 127.0.0.1 with: adb reverse tcp:5005 tcp:5005")]
    public string host = "127.0.0.1";
    public int port = 5005;

    [Header("Streaming")]
    [Range(20.0f, 120.0f)]
    public float sendRateHz = 72.0f;

    private InputDevice leftDevice;
    private InputDevice rightDevice;
    private uint sequence;
    private double nextSendTime;

    private readonly object packetLock = new object();
    private byte[] latestPacket;
    private readonly AutoResetEvent packetReady = new AutoResetEvent(false);
    private Thread networkThread;
    private volatile bool running;
    private volatile bool connected;
    private bool previousConnected;
    private string workerHost;
    private int workerPort;

    private struct HandData
    {
        public Vector3 position;
        public Quaternion rotation;
        public float grip;
        public float trigger;
        public bool valid;
    }

    private void Start()
    {
        Application.runInBackground = true;
        workerHost = host;
        workerPort = port;
        running = true;
        networkThread = new Thread(NetworkWorker)
        {
            IsBackground = true,
            Name = "QuestRobotTcpSender"
        };
        networkThread.Start();
    }

    private void Update()
    {
        if (connected != previousConnected)
        {
            previousConnected = connected;
            Debug.Log(connected
                ? "Connected to robot teleop server"
                : "Disconnected from robot teleop server");
        }

        double now = Time.realtimeSinceStartupAsDouble;
        if (now < nextSendTime)
        {
            return;
        }
        nextSendTime = now + 1.0 / Math.Max(1.0f, sendRateHz);

        HandData left = ReadHand(XRNode.LeftHand, ref leftDevice);
        HandData right = ReadHand(XRNode.RightHand, ref rightDevice);
        byte[] packet = BuildPacket(now, left, right);

        lock (packetLock)
        {
            latestPacket = packet;
        }
        packetReady.Set();
    }

    private static HandData ReadHand(XRNode node, ref InputDevice device)
    {
        if (!device.isValid)
        {
            device = InputDevices.GetDeviceAtXRNode(node);
        }

        HandData data = new HandData
        {
            position = Vector3.zero,
            rotation = Quaternion.identity,
            grip = 0.0f,
            trigger = 0.0f,
            valid = false
        };

        if (!device.isValid)
        {
            return data;
        }

        bool hasPosition = device.TryGetFeatureValue(CommonUsages.devicePosition, out data.position);
        bool hasRotation = device.TryGetFeatureValue(CommonUsages.deviceRotation, out data.rotation);
        device.TryGetFeatureValue(CommonUsages.grip, out data.grip);
        device.TryGetFeatureValue(CommonUsages.trigger, out data.trigger);
        data.valid = hasPosition && hasRotation;
        return data;
    }

    private byte[] BuildPacket(double deviceTimeSeconds, HandData left, HandData right)
    {
        using (MemoryStream stream = new MemoryStream(104))
        using (BinaryWriter writer = new BinaryWriter(stream))
        {
            writer.Write(Magic);
            writer.Write(ProtocolVersion);
            writer.Write(sequence++);
            writer.Write(0u); // reserved
            writer.Write(deviceTimeSeconds);
            WriteHand(writer, left);
            WriteHand(writer, right);
            return stream.ToArray();
        }
    }

    private static void WriteHand(BinaryWriter writer, HandData hand)
    {
        writer.Write(hand.position.x);
        writer.Write(hand.position.y);
        writer.Write(hand.position.z);
        writer.Write(hand.rotation.x);
        writer.Write(hand.rotation.y);
        writer.Write(hand.rotation.z);
        writer.Write(hand.rotation.w);
        writer.Write(hand.grip);
        writer.Write(hand.trigger);
        writer.Write(hand.valid ? 1u : 0u);
    }

    private void NetworkWorker()
    {
        TcpClient client = null;
        NetworkStream stream = null;

        while (running)
        {
            try
            {
                if (client == null || !client.Connected)
                {
                    CloseClient(ref client, ref stream);
                    client = ConnectWithTimeout(workerHost, workerPort, 1000);
                    client.NoDelay = true;
                    client.SendTimeout = 100;
                    stream = client.GetStream();
                    connected = true;
                }

                packetReady.WaitOne(100);
                if (!running)
                {
                    break;
                }

                byte[] packet;
                lock (packetLock)
                {
                    packet = latestPacket;
                }

                if (packet != null)
                {
                    stream.Write(packet, 0, packet.Length);
                }
            }
            catch
            {
                connected = false;
                CloseClient(ref client, ref stream);
                Thread.Sleep(500);
            }
        }

        connected = false;
        CloseClient(ref client, ref stream);
    }

    private static TcpClient ConnectWithTimeout(string targetHost, int targetPort, int timeoutMs)
    {
        TcpClient client = new TcpClient();
        IAsyncResult result = client.BeginConnect(targetHost, targetPort, null, null);
        bool success = result.AsyncWaitHandle.WaitOne(timeoutMs);
        if (!success)
        {
            client.Close();
            throw new TimeoutException("TCP connection timeout");
        }
        client.EndConnect(result);
        return client;
    }

    private static void CloseClient(ref TcpClient client, ref NetworkStream stream)
    {
        try { stream?.Close(); } catch { }
        try { client?.Close(); } catch { }
        stream = null;
        client = null;
    }

    private void OnDestroy()
    {
        StopWorker();
    }

    private void OnApplicationQuit()
    {
        StopWorker();
    }

    private void StopWorker()
    {
        if (!running)
        {
            return;
        }
        running = false;
        packetReady.Set();
        if (networkThread != null && networkThread.IsAlive)
        {
            networkThread.Join(1000);
        }
        networkThread = null;
    }
}
