# Real-time file streaming

This project involves the development of a **file streaming system** designed specifically for **video** and **audio** files, using a protocol inspired on BitTorrent. The system comprises two main components: **Servers** (acting as **Trackers**) and **Clients**. Each Client will provide an GUI that can be used to interact with the system and will show the available files on the network. 

- The **Clients** represent users who wish to stream files within the system, contributing to the exchange of file chunks.
- The **Server (Tracker)** functions as an intermediary between Clients, responsible for managing the file-streaming process by coordinating peer connections. Like clients, the server will also have files (audio and video) available for streaming.
 
This implementation provides a simplified version of the BitTorrent protocol, focusing on core functionalities like **peer discovery** and **file chunk exchange**.

## Server (Tracker)

The Server has the following responsibilities:
- **Peer Management**: 
  - Maintain a list of active clients participating in the file-streaming process.
  - Keep track of the files available at each client (file name, size).
- **Connection coordination**: Help clients connect to each other for file chunk exchange by providing an optimal path for data transfer.
- **File streaming**: store files data that can be streamed by clients.
- **Communication protocol**: implement a protocol to communicate with clients.
  - Register new clients.
  - Update file lists.
  - Provide peer lists to clients.
  - Handle client disconnections.

## Client

There are two types of clients: **audio** and **video**. Each client will be able to show through the GUI only audio or video files, respectively and both will provide a GUI to stream files, with the following responsibilities:
- **File streaming**: 
  - Stream files from the network.
  - Display the streamed file.
- **Peer discovery**:
  - Connect to the Server to register its availability.
  - Share its file list with the Server.
  - Request peer lists that have the desired file from the Server.

Presentation: [Rust-eze presentation](https://www.canva.com/design/DAGetvV6idQ/vh7g-1nROe4iQjRrYg-ZGg/view?utm_content=DAGetvV6idQ&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=h19bc15de3d)