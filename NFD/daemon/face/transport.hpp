/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2015,  Regents of the University of California,
 *                           Arizona Board of Regents,
 *                           Colorado State University,
 *                           University Pierre & Marie Curie, Sorbonne University,
 *                           Washington University in St. Louis,
 *                           Beijing Institute of Technology,
 *                           The University of Memphis.
 *
 * This file is part of NFD (Named Data Networking Forwarding Daemon).
 * See AUTHORS.md for complete list of NFD authors and contributors.
 *
 * NFD is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NFD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NFD, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef NFD_DAEMON_FACE_TRANSPORT_HPP
#define NFD_DAEMON_FACE_TRANSPORT_HPP

#include "common.hpp"

#include "face-counters.hpp"
#include "face-log.hpp"

namespace nfd {
namespace face {

class LpFace;
class LinkService;

/** \brief indicates the state of a transport
 */
enum class TransportState {
  NONE,
  UP, ///< the transport is up
  DOWN, ///< the transport is down temporarily, and is being recovered
  CLOSING, ///< the transport is requested to be closed
  FAILED, ///< the transport is being closed due to a failure
  CLOSED ///< the transport is closed, and can be safely deallocated
};

std::ostream&
operator<<(std::ostream& os, TransportState state);

/** \brief indicates the transport has no limit on payload size
 */
const ssize_t MTU_UNLIMITED = -1;

/** \brief the lower part of an LpFace
 *  \sa LpFace
 */
class Transport : noncopyable
{
public:
  /** \brief identifies an endpoint on the link
   */
  typedef uint64_t EndpointId;

  /** \brief stores a packet along with the remote endpoint
   */
  class Packet
  {
  public:
    Packet() = default;

    explicit
    Packet(Block&& packet);

  public:
    /** \brief the packet as a TLV block
     */
    Block packet;

    /** \brief identifies the remote endpoint
     *
     *  This ID is only meaningful in the context of the same Transport.
     *  Incoming packets from the same remote endpoint have the same EndpointId,
     *  and incoming packets from different remote endpoints have different EndpointIds.
     */
    EndpointId remoteEndpoint;
  };

  Transport();

  virtual
  ~Transport();

public:
  /** \brief set Face and LinkService for Transport
   *  \pre setFaceAndLinkService has not been called
   */
  void
  setFaceAndLinkService(LpFace& face, LinkService& service);

  /** \return Face to which this Transport is attached
   */
  const LpFace*
  getFace() const;

  /** \return LinkService to which this Transport is attached
   */
  const LinkService*
  getLinkService() const;

  /** \return LinkService to which this Transport is attached
   */
  LinkService*
  getLinkService();

public: // upper interface
  /** \brief request the transport to be closed
   *
   *  This operation is effective only if transport is in UP or DOWN state,
   *  otherwise it has no effect.
   *  The transport changes state to CLOSING, and performs cleanup procedure.
   *  The state will be changed to CLOSED when cleanup is complete, which may
   *  happen synchronously or asynchronously.
   */
  void
  close();

  /** \brief send a link-layer packet
   *  \note This operation has no effect if \p getState() is neither UP nor DOWN
   *  \warning undefined behavior if packet size exceeds MTU limit
   */
  void
  send(Packet&& packet);

protected: // upper interface to be invoked by subclass
  /** \brief receive a link-layer packet
   *  \warning undefined behavior if packet size exceeds MTU limit
   */
  void
  receive(Packet&& packet);

public: // static properties
  /** \return a FaceUri representing local endpoint
   */
  FaceUri
  getLocalUri() const;

  /** \return a FaceUri representing remote endpoint
   */
  FaceUri
  getRemoteUri() const;

  /** \return whether face is local or non-local for scope control purpose
   */
  ndn::nfd::FaceScope
  getScope() const;

  /** \return face persistency setting
   */
  ndn::nfd::FacePersistency
  getPersistency() const;

  /** \brief changes face persistency setting
   */
  void
  setPersistency(ndn::nfd::FacePersistency persistency);

  /** \return whether face is point-to-point or multi-access
   */
  ndn::nfd::LinkType
  getLinkType() const;

  /** \return maximum payload size
   *  \retval MTU_UNLIMITED transport has no limit on payload size
   *
   *  This size is the maximum packet size that can be sent or received through this transport.
   *
   *  For a datagram-based transport, this is typically the Maximum Transmission Unit (MTU),
   *  after the overhead of headers introduced by the transport has been accounted for.
   *  For a stream-based transport, this is typically unlimited (MTU_UNLIMITED).
   */
  ssize_t
  getMtu() const;

public: // dynamic properties
  /** \return transport state
   */
  TransportState
  getState() const;

  /** \brief signals when transport state changes
   */
  signal::Signal<Transport, TransportState/*old*/, TransportState/*new*/> afterStateChange;

protected: // properties to be set by subclass
  void
  setLocalUri(const FaceUri& uri);

  void
  setRemoteUri(const FaceUri& uri);

  void
  setScope(ndn::nfd::FaceScope scope);

  void
  setLinkType(ndn::nfd::LinkType linkType);

  void
  setMtu(ssize_t mtu);

  /** \brief set transport state
   *
   *  Only the following transitions are valid:
   *  UP->DOWN, DOWN->UP, UP/DOWN->CLOSING/FAILED, CLOSING/FAILED->CLOSED
   *
   *  \throw std::runtime_error transition is invalid.
   */
  void
  setState(TransportState newState);

protected: // to be overridden by subclass
  /** \brief invoked before persistency is changed
   *  \throw std::invalid_argument new persistency is not supported
   *  \throw std::runtime_error transition is disallowed
   *
   *  Base class implementation does nothing.
   */
  virtual void
  beforeChangePersistency(ndn::nfd::FacePersistency newPersistency)
  {
  }

  /** \brief performs Transport specific operations to close the transport
   *
   *  This is invoked once by \p close() after changing state to CLOSING.
   *  It will not be invoked by Transport class if the transport is already CLOSING or CLOSED.
   *
   *  When the cleanup procedure is complete, this method should change state to CLOSED.
   *  This transition can happen synchronously or asynchronously.
   */
  virtual void
  doClose() = 0;

private: // to be overridden by subclass
  /** \brief performs Transport specific operations to send a packet
   *  \param packet the packet, which must be a well-formed TLV block
   *  \pre state is either UP or DOWN
   */
  virtual void
  doSend(Packet&& packet) = 0;

private:
  LpFace* m_face;
  LinkService* m_service;
  FaceUri m_localUri;
  FaceUri m_remoteUri;
  ndn::nfd::FaceScope m_scope;
  ndn::nfd::FacePersistency m_persistency;
  ndn::nfd::LinkType m_linkType;
  ssize_t m_mtu;
  TransportState m_state;
  LinkLayerCounters* m_counters; // TODO#3177 change into LinkCounters
};

inline const LpFace*
Transport::getFace() const
{
  return m_face;
}

inline const LinkService*
Transport::getLinkService() const
{
  return m_service;
}

inline LinkService*
Transport::getLinkService()
{
  return m_service;
}

inline FaceUri
Transport::getLocalUri() const
{
  return m_localUri;
}

inline void
Transport::setLocalUri(const FaceUri& uri)
{
  m_localUri = uri;
}

inline FaceUri
Transport::getRemoteUri() const
{
  return m_remoteUri;
}

inline void
Transport::setRemoteUri(const FaceUri& uri)
{
  m_remoteUri = uri;
}

inline ndn::nfd::FaceScope
Transport::getScope() const
{
  return m_scope;
}

inline void
Transport::setScope(ndn::nfd::FaceScope scope)
{
  m_scope = scope;
}

inline ndn::nfd::FacePersistency
Transport::getPersistency() const
{
  return m_persistency;
}

inline void
Transport::setPersistency(ndn::nfd::FacePersistency persistency)
{
  this->beforeChangePersistency(persistency);
  m_persistency = persistency;
}

inline ndn::nfd::LinkType
Transport::getLinkType() const
{
  return m_linkType;
}

inline void
Transport::setLinkType(ndn::nfd::LinkType linkType)
{
  m_linkType = linkType;
}

inline ssize_t
Transport::getMtu() const
{
  return m_mtu;
}

inline void
Transport::setMtu(ssize_t mtu)
{
  BOOST_ASSERT(mtu == MTU_UNLIMITED || mtu > 0);
  m_mtu = mtu;
}

inline TransportState
Transport::getState() const
{
  return m_state;
}

std::ostream&
operator<<(std::ostream& os, const FaceLogHelper<Transport>& flh);

template<typename T>
typename std::enable_if<std::is_base_of<Transport, T>::value &&
                        !std::is_same<Transport, T>::value, std::ostream&>::type
operator<<(std::ostream& os, const FaceLogHelper<T>& flh)
{
  return os << FaceLogHelper<Transport>(flh.obj);
}

} // namespace face
} // namespace nfd

#endif // NFD_DAEMON_FACE_TRANSPORT_HPP
