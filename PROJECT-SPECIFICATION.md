# File sharing with BitTorrent protocol

This project involves the development of a **file sharing system** designed specifically for **video** and **audio** files, using the BitTorrent protocol. The system comprises two main components: **Servers** (acting as **Trackers**) and **Clients**. Each Client will be able to connect to an interface that shows a list of files which can be downloaded from other Clients.

- The **Server (Tracker)** functions as an intermediary between Clients, responsible for managing the file-sharing process by coordinating peer connections.
- The **Clients** represent users who wish to share or download files within the system, contributing to the distributed exchange of file chunks.

This implementation provides a simplified version of the BitTorrent protocol, focusing on core functionalities like peer discovery, file chunk exchange.

## Server (Tracker)

The Server has the following responsibilities:
- **Peer Management**: 
  - Maintain a list of active clients participating in the file-sharing process.
  - Keep track of the files available at each client (file name, size).
- **Connection coordination**: Help clients connect to each other for file chunk exchange by providing an optimal path for data transfer.
- **Communication protocol**: implement a protocol to communicate with clients.
  - Register new clients.
  - Update file lists.
  - Provide peer lists to clients.
  - Handle client disconnections.

## Client

There are two types of clients: **audio** and **video**. Each client will be able to show through the GUI only audio or video files, respectively and both will act as a **downloader** and an **uploader** of files, with the following responsibilities:
- **File sharing**: 
  - Download files from other clients.
  - Upload files to other clients.
- **Peer discovery**:
  - Connect to the Server to register its availability.
  - Share its file list with the Server.
  - Request peer lists that have the desired file from the Server.
- **File chunk management**: Manage which chunks of the file has been downloaded and which chunks are missing.

Presentation: [Rust-eze presentation](https://www.canva.com/design/DAGetvV6idQ/vh7g-1nROe4iQjRrYg-ZGg/view?utm_content=DAGetvV6idQ&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=h19bc15de3d)
