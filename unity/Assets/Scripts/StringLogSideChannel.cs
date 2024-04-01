using UnityEngine;
using Unity.MLAgents.SideChannels;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Globalization;
using Random=UnityEngine.Random;

public class StringLogSideChannel : SideChannel
{
    
    private Vector3 receivedPosition = new Vector3(0f, 6.41f, 3f);
    private Quaternion receivedRotation = new Quaternion(0f, 0f, 0f, 1f);
    private CultureInfo info = CultureInfo.GetCultureInfo("en-US");
    
    public StringLogSideChannel()
    {
        ChannelId = new Guid("621f0a70-4f87-11ea-a6bf-784f4387d1f7");
    }

    public Vector3 getPosition(){
        return receivedPosition;
    }

    public Quaternion getRotation(){
        return receivedRotation;
    }

    public void setRandom(){
        receivedPosition = new Vector3(Random.Range(-1.5f, 1.5f), 6.41f, Random.Range(1.5f, 3f));
        receivedRotation = Quaternion.Euler(0f, Random.Range(-45f, 45f), 0f);
        Debug.Log("Set position randomly");
    }

    public void setNext(float posX, float posZ, float rotY)
    {
        receivedPosition = new Vector3(posX, 6.5f, posZ);
        receivedRotation = Quaternion.Euler(0, rotY, 0);
    }

    protected override void OnMessageReceived(IncomingMessage msg)
    {
        var receivedString = msg.ReadString();
        Debug.Log("From Python : " + receivedString);
        List<string> result = receivedString.Split('/').ToList();        
        receivedPosition = new Vector3(float.Parse(result[0], info), 6.5f, float.Parse(result[1], info));
        receivedRotation = Quaternion.Euler(0, float.Parse(result[2], info), 0);
        
        Debug.Log("Pos: " + receivedPosition + " , Rot:" + receivedRotation);
    }

    public void SendDebugStatementToPython(string logString, string stackTrace, LogType type)
    {
        if (type == LogType.Error)
        {
            var stringToSend = type.ToString() + ": " + logString + "\n" + stackTrace;
            using (var msgOut = new OutgoingMessage())
            {
                msgOut.WriteString(stringToSend);
                QueueMessageToSend(msgOut);
            }
        }
    }

    public void SendMessageToPython(string message)
    {
        using var msgOut = new OutgoingMessage();
        msgOut.WriteString(message);
        QueueMessageToSend(msgOut);
    }
}