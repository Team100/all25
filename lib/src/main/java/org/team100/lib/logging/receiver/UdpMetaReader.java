package org.team100.lib.logging.receiver;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;

import org.team100.lib.logging.primitive.UdpSender;
import org.team100.lib.util.Util;
import org.team100.lib.logging.primitive.UdpPrimitiveProtocol.ProtocolException;

public class UdpMetaReader implements Runnable {
    private final UdpMetaDecoder m_decoder;

    /** nullable */
    private final DatagramChannel m_channel;

    /** TODO: maybe this should be the "protocol"'s buffer' */
    private final ByteBuffer m_buffer;

    public UdpMetaReader(UdpMetaDecoder decoder) {
        m_decoder = decoder;
        m_channel = makeChannel(UdpSender.kmetadataPort);
        m_buffer = ByteBuffer.allocateDirect(UdpSender.MTU);
        // big-endian is the default, but just to make it clear...
        m_buffer.order(ByteOrder.BIG_ENDIAN);
    }

    @Override
    public void run() {
        Util.println("meta reader running...");
        while (true) {
            try {
                m_buffer.clear();
                // TODO: replace this with socket.read with a timeout
                m_channel.receive(m_buffer);
                m_buffer.limit(m_buffer.position());
                m_buffer.position(0);
                if (!m_decoder.validateTimestamp(m_buffer)) {
                    Util.warn("meta timestamp is bad, bail");
                    return;
                }
                while (m_buffer.remaining() > 0) {
                    m_decoder.decode(m_buffer);
                }
            } catch (IOException | ProtocolException e) {
                e.printStackTrace();
            }
        }
    }

    private static DatagramChannel makeChannel(int port) {
        try {
            DatagramChannel channel = DatagramChannel.open();
            channel.configureBlocking(true);
            InetSocketAddress sockAddr = new InetSocketAddress(port);
            channel.bind(sockAddr);
            return channel;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }
}
