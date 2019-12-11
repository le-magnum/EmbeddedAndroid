package com.example.esdandroid;

import java.io.IOException;

public class TestTcp {

    public static void main(String[] args) throws IOException {
            TcpClient client = new TcpClient();
            client.startConnection("10.10.220.74", 5000);
            String response = client.sendMessage("hello server");
            System.out.println(response);
    }

}
