#include <helpers/rabbit_helper.hpp>

//TODO: Create Rabbit_Helper on the heap instead of the stack. Hint: use operator "new"
//      This connection is likely long living but constantly referenced.
#include <amqpcpp/linux_tcp.h>

/*
class RabbitHelper : public AMQP::TcpHandler
{
    virtual void onAttached(AMQP::TcpConnection *connection) override
    {
        // @todo
        //  add your own implementation, for example initialize things
        //  to handle the connection.
    }

    virtual void onConnected(AMQP::TcpConnection *connection) override
    {
        // @todo
        //  add your own implementation (probably not needed)
    }

    virtual bool onSecured(AMQP::TcpConnection *connection, const SSL *ssl) override
    {
        // @todo
        //  add your own implementation, for example by reading out the
        //  certificate and check if it is indeed yours
        return true;
    }

    virtual void onReady(AMQP::TcpConnection *connection) override
    {
        // @todo
        //  add your own implementation, for example by creating a channel
        //  instance, and start publishing or consuming
    }

    virtual void onError(AMQP::TcpConnection *connection, const char *message) override
    {
        // @todo
        //  add your own implementation, for example by reporting the error
        //  to the user of your program and logging the error
    }

    virtual void onClosed(AMQP::TcpConnection *connection) override
    {
        // @todo
        //  add your own implementation (probably not necessary, but it could
        //  be useful if you want to do some something immediately after the
        //  amqp connection is over, but do not want to wait for the tcp
        //  connection to shut down
    }

    virtual void onLost(AMQP::TcpConnection *connection) override
    {
        // @todo
        //  add your own implementation (probably not necessary)
    }

    virtual void onDetached(AMQP::TcpConnection *connection) override
    {
        // @todo
        //  add your own implementation, like cleanup resources or exit the application
    }

    virtual void monitor(AMQP::TcpConnection *connection, int fd, int flags) override
    {
        // @todo
        //  add your own implementation, for example by adding the file
        //  descriptor to the main application event loop (like the select() or
        //  poll() loop). When the event loop reports that the descriptor becomes
        //  readable and/or writable, it is up to you to inform the AMQP-CPP
        //  library that the filedescriptor is active by calling the
        //  connection->process(fd, flags) method.
    }
};

// create an instance of your own tcp handler
RabbitHelper rabbit;

// address of the server
AMQP::Address address("amqp://guest:guest@localhost/vhost");

// create a AMQP connection object
AMQP::TcpConnection connection(&rabbit, address);

// and create a channel
AMQP::TcpChannel channel(&connection);

// use the channel object to call the AMQP method you like
//channel.declareExchange("my-exchange", AMQP::fanout);
//channel.declareQueue("my-queue");
//channel.bindQueue("my-exchange", "my-queue", "my-routing-key");
//    channel.declareQueue(AMQP::exclusive).onSuccess([&connection](const std::string &name, uint32_t messagecount, uint32_t consumercount) {

        // report the name of the temporary queue
        std::cout << "declared queue " << name << std::endl;
    });
*/
