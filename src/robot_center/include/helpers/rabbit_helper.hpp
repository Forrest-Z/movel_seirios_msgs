#include <ev.h>
#include <amqpcpp.h>
#include <amqpcpp/libuv.h>

//TODO: Remove AMQP-CPP TCP lib so that binary can use native Linux's TCP lib without carrying duplicate. But more work needed (and hence errors) to help setupup network connections and socket buffering.
// TODO: Implement custom Event handling so that custom heartbeats could be configured. Would be nice to go to libuv, which is asynchronous and have a heartbeat every 30s.

class MyTcpHandler : public AMQP::TcpHandler
{
};
/**
 *  Parse data that was received from RabbitMQ
 *
 *  Every time that data comes in from RabbitMQ, you should call this method to parse
 *  the incoming data, and let it handle by the AMQP-CPP library. This method returns
 *  the number of bytes that were processed.
 *
 *  If not all bytes could be processed because it only contained a partial frame,
 *  you should call this same method later on when more data is available. The
 *  AMQP-CPP library does not do any buffering, so it is up to the caller to ensure
 *  that the old data is also passed in that later call.
 *
 *  @param  buffer      buffer to decode
 *  @param  size        size of the buffer to decode
 *  @return             number of bytes that were processed
 */

/*
size_t parse(char *buffer, size_t size)
{
    // TODO: use Connection::expected: Number of bytes lib prefer to receive. Does not make sense to call parse() with an adjusted smaller buffer. OR
    // use Connection::maxFrame(): Allocate a buffer up to max size so that buffer is reusable.

    //return _implementation.parse(buffer, size);
}
*/

class RabbitHelper : public AMQP::LibUvHandler
{
  private:
    /**
     *  Method that is called by the AMQP library when a new connection
     *  is associated with the handler. This is the first call to your handler
     *  @param  connection      The connection that is attached to the handler
     */
    virtual void onAttached(AMQP::TcpConnection *connection) override
    {
    }

    /**
     *  Method that is called by the AMQP library when the TCP connection
     *  has been established. After this method has been called, the library
     *  still has take care of setting up the optional TLS layer and of
     *  setting up the AMQP connection on top of the TCP layer., This method
     *  is always paired with a later call to onLost().
     *  @param  connection      The connection that can now be used
     */
    virtual void onConnected(AMQP::TcpConnection *connection) override
    {
    }

    /**
     *  Method that is called when the secure TLS connection has been established.
     *  This is only called for amqps:// connections. It allows you to inspect
     *  whether the connection is secure enough for your liking (you can
     *  for example check the server certicate). The AMQP protocol still has
     *  to be started.
     *  @param  connection      The connection that has been secured
     *  @param  ssl             SSL structure from openssl library
     *  @return bool            True if connection can be used
     */
    virtual bool onSecured(AMQP::TcpConnection *connection, const SSL *ssl) override
    {
    }

    /**
     *  Method that is called by the AMQP library when the login attempt
     *  succeeded. After this the connection is ready to use.
     *  @param  connection      The connection that can now be used
     */
    virtual void onReady(AMQP::TcpConnection *connection) override
    {
    }

    /**
     *  Method that is called by the AMQP library when a fatal error occurs
     *  on the connection, for example because data received from RabbitMQ
     *  could not be recognized, or the underlying connection is lost. This
     *  call is normally followed by a call to onLost() (if the error occured
     *  after the TCP connection was established) and onDetached().
     *  @param  connection      The connection on which the error occured
     *  @param  message         A human readable error message
     */
    virtual void onError(AMQP::TcpConnection *connection, const char *message) override
    {
    }

    /**
     *  Method that is called when the AMQP protocol is ended. This is the
     *  counter-part of a call to connection.close() to graceful shutdown
     *  the connection. Note that the TCP connection is at this time still
     *  active, and you will also receive calls to onLost() and onDetached()
     *  @param  connection      The connection over which the AMQP protocol ended
     */
    virtual void onClosed(AMQP::TcpConnection *connection) override
    {
    }

    /**
     *  Method that is called when the TCP connection was closed or lost.
     *  This method is always called if there was also a call to onConnected()
     *  @param  connection      The connection that was closed and that is now unusable
     */
    virtual void onLost(AMQP::TcpConnection *connection) override
    {
    }

    /**
     *  Final method that is called. This signals that no further calls to your
     *  handler will be made about the connection.
     *  @param  connection      The connection that can be destructed
     */
    virtual void onDetached(AMQP::TcpConnection *connection) override
    {
    }

    /**
     *  Method that is called by the AMQP-CPP library when it wants to interact
     *  with the main event loop. The AMQP-CPP library is completely non-blocking,
     *  and only make "write()" or "read()" system calls when it knows in advance
     *  that these calls will not block. To register a filedescriptor in the
     *  event loop, it calls this "monitor()" method with a filedescriptor and
     *  flags telling whether the filedescriptor should be checked for readability
     *  or writability.
     *
     *  @param  connection      The connection that wants to interact with the event loop
     *  @param  fd              The filedescriptor that should be checked
     *  @param  flags           Bitwise or of AMQP::readable and/or AMQP::writable
    virtual void monitor(AMQP::TcpConnection *connection, int fd, int flags) override
    {
    }
     */

  protected:

  public:
    RabbitHelper(uv_loop_t *loop) : AMQP::LibUvHandler(loop) {};
    virtual ~RabbitHelper() = default;


    void init();
};
