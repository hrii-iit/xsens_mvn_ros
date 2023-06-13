#include "xsens_mvn_ros/Socket.h"

Socket::Socket(IpProtocol protocol) 
{
    m_protocol = protocol;
    fromlen = sizeof(struct sockaddr_in);

    if (m_protocol == IP_UDP) {
        m_sock = socket(AF_INET, SOCK_DGRAM, 0);
    }
    else if (m_protocol == IP_TCP) {
        m_sock = socket(AF_INET, SOCK_STREAM, 0);
    }
    else {
        throw std::invalid_argument("Unknown protocol");
    }

}

Socket::~Socket()
{
    close(m_sock);
}

bool Socket::bind(uint16_t port) 
{
    if (port < 2000)
        throw std::invalid_argument("Port number should be bigger than 2000");
    
    memset((char *) &m_my_addr, 0, sizeof(m_my_addr));

    m_my_addr.sin_family = AF_INET;
    m_my_addr.sin_addr.s_addr = INADDR_ANY;
    m_my_addr.sin_port = htons(port);

    if (::bind(m_sock, (struct sockaddr *) &m_my_addr, sizeof(m_my_addr)) < 0) {
        std::cout << "socket binding failed" << std::endl;
        return false;
    }
        
    return true;
}

int Socket::read(char* buffer, size_t size) 
{
    int n = -1;
    memset(buffer,0, size);
    
    if (m_protocol == IP_UDP) {
        n = recvfrom(m_sock, buffer, size, 0, (struct sockaddr*)&m_addr_from, &fromlen);
    }
    else {
        std::cout << "message not received " << std::endl;
    }
    return n;
}
