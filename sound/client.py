
import time

from pythonosc import udp_client


if __name__ == "__main__":
    client = udp_client.SimpleUDPClient( "127.0.0.1", 5005 )
    while True:
        client.send_message("/midi1", 60 )
        client.send_message("/midi2", 64 )
        client.send_message("/midi3", 67 )
        client.send_message("/volume", 0.5)
        time.sleep(5)
        client.send_message("/volume", 1)
        time.sleep(5)

    # while True :
    #     client.send_message( "/reset", 0 )

    #     # middle C




    #     time.sleep(0.5)
    
