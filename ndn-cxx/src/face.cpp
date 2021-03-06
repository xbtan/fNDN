/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2013-2015 Regents of the University of California.
 *
 * This file is part of ndn-cxx library (NDN C++ library with eXperimental eXtensions).
 *
 * ndn-cxx library is free software: you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * ndn-cxx library is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 *
 * You should have received copies of the GNU General Public License and GNU Lesser
 * General Public License along with ndn-cxx, e.g., in COPYING.md file.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * See AUTHORS.md for complete list of ndn-cxx authors and contributors.
 */

#include "face.hpp"
#include "detail/face-impl.hpp"

#include "encoding/tlv.hpp"
#include "security/key-chain.hpp"
#include "security/signing-helpers.hpp"
#include "util/time.hpp"
#include "util/random.hpp"
#include "util/face-uri.hpp"
#include <fstream>
namespace ndn {

Face::Face()
  : m_internalIoService(new boost::asio::io_service())
  , m_ioService(*m_internalIoService)
  , m_internalKeyChain(new KeyChain())
  , m_impl(new Impl(*this))
{
  construct(*m_internalKeyChain);
}

Face::Face(boost::asio::io_service& ioService)
  : m_ioService(ioService)
  , m_internalKeyChain(new KeyChain())
  , m_impl(new Impl(*this))
{
  construct(*m_internalKeyChain);
}

Face::Face(const std::string& host, const std::string& port/* = "6363"*/)
  : m_internalIoService(new boost::asio::io_service())
  , m_ioService(*m_internalIoService)
  , m_internalKeyChain(new KeyChain())
  , m_impl(new Impl(*this))
{
  construct(make_shared<TcpTransport>(host, port), *m_internalKeyChain);
}

Face::Face(const shared_ptr<Transport>& transport)
  : m_internalIoService(new boost::asio::io_service())
  , m_ioService(*m_internalIoService)
  , m_internalKeyChain(new KeyChain())
  , m_impl(new Impl(*this))
{
  construct(transport, *m_internalKeyChain);
}

Face::Face(const shared_ptr<Transport>& transport,
           boost::asio::io_service& ioService)
  : m_ioService(ioService)
  , m_internalKeyChain(new KeyChain())
  , m_impl(new Impl(*this))
{
  construct(transport, *m_internalKeyChain);
}

Face::Face(shared_ptr<Transport> transport,
           boost::asio::io_service& ioService,
           KeyChain& keyChain)
  : m_ioService(ioService)
  , m_internalKeyChain(nullptr)
  , m_impl(new Impl(*this))
{
  construct(transport, keyChain);
}

void
Face::construct(KeyChain& keyChain)
{
  // transport=unix:///var/run/nfd.sock
  // transport=tcp://localhost:6363

  ConfigFile config;
  const auto& transportType = config.getParsedConfiguration()
                                .get_optional<std::string>("transport");
  if (!transportType) {
    // transport not specified, use default Unix transport.
    construct(UnixTransport::create(config), keyChain);
    return;
  }

  unique_ptr<util::FaceUri> uri;
  try {
    uri.reset(new util::FaceUri(*transportType));
  }
  catch (const util::FaceUri::Error& error) {
    BOOST_THROW_EXCEPTION(ConfigFile::Error(error.what()));
  }

  const std::string protocol = uri->getScheme();

  if (protocol == "unix") {
    construct(UnixTransport::create(config), keyChain);
  }
  else if (protocol == "tcp" || protocol == "tcp4" || protocol == "tcp6") {
    construct(TcpTransport::create(config), keyChain);
  }
  else {
    BOOST_THROW_EXCEPTION(ConfigFile::Error("Unsupported transport protocol \"" + protocol + "\""));
  }
}

void
Face::construct(shared_ptr<Transport> transport, KeyChain& keyChain)
{
  m_nfdController.reset(new nfd::Controller(*this, keyChain));

  m_transport = transport;

  m_ioService.post([=] { m_impl->ensureConnected(false); });
}

Face::~Face() = default;

const PendingInterestId*
Face::expressInterest(const Interest& interest,
                      const DataCallback& afterSatisfied,
                      const NackCallback& afterNacked,
                      const TimeoutCallback& afterTimeout)
{
  
    /*size_t FC = interest.getFC();
    size_t EC = interest.getEC();
    std::string flowname = interest.getName().toUri();
    while(FC < EC)
    {
        Interest temp_interest(Name(flowname).appendSegment(FC)); 
         FC++;
         temp_interest.setInterestLifetime(time::milliseconds(1000));
             temp_interest.setMustBeFresh(true);
       shared_ptr<Interest> interestToExpress = make_shared<Interest>(temp_interest);
     
       // Use `interestToExpress` to avoid wire format creation for the original Interest
       if (interestToExpress->wireEncode().size() > MAX_NDN_PACKET_SIZE) {
         BOOST_THROW_EXCEPTION(Error("Interest size exceeds maximum limit"));
       }
     
       // If the same ioService thread, dispatch directly calls the method
       m_impl->MyasyncExpressInterest(interestToExpress, afterSatisfied,
                                                              afterNacked, afterTimeout);
    }*/

  shared_ptr<Interest> interestToExpress = make_shared<Interest>(interest);

  // Use `interestToExpress` to avoid wire format creation for the original Interest
  if (interestToExpress->wireEncode().size() > MAX_NDN_PACKET_SIZE) {
    BOOST_THROW_EXCEPTION(Error("Interest size exceeds maximum limit"));
  }

  // If the same ioService thread, dispatch directly calls the method
  m_ioService.dispatch([=] { m_impl->asyncExpressInterest(interestToExpress, afterSatisfied,
                                                          afterNacked, afterTimeout); });

  return reinterpret_cast<const PendingInterestId*>(interestToExpress.get());
}


const PendingInterestId*
Face::expressInterest(const Interest& interest,
                      const OnData& onData,
                      const OnTimeout& onTimeout)
{
  return this->expressInterest(
    interest,
    [onData] (const Interest& interest, const Data& data) {
      if (onData != nullptr) {
        onData(interest, const_cast<Data&>(data));
      }
    },
    [onTimeout] (const Interest& interest, const lp::Nack& nack) {
      if (onTimeout != nullptr) {
        onTimeout(interest);
      }
    },
    onTimeout
  );
}

const PendingInterestId*
Face::expressInterest(const Name& name,
                      const Interest& tmpl,
                      const OnData& onData, const OnTimeout& onTimeout/* = nullptr*/)
{
  return expressInterest(Interest(tmpl)
                         .setName(name)
                         .setNonce(0),
                         onData, onTimeout);
}

void
Face::put(const Data& data)
{
  // Use original `data`, since wire format should already exist for the original Data
  if (data.wireEncode().size() > MAX_NDN_PACKET_SIZE)
    BOOST_THROW_EXCEPTION(Error("Data size exceeds maximum limit"));

  shared_ptr<const Data> dataPtr;
  try {
    dataPtr = data.shared_from_this();
  }
  catch (const bad_weak_ptr& e) {
    std::cerr << "Face::put WARNING: the supplied Data should be created using make_shared<Data>()"
              << std::endl;
    dataPtr = make_shared<Data>(data);
  }

  // If the same ioService thread, dispatch directly calls the method
  m_ioService.dispatch([=] { m_impl->asyncPutData(dataPtr); });
}

void
Face::put(const lp::Nack& nack)
{
  m_ioService.dispatch([=] { m_impl->asyncPutNack(make_shared<lp::Nack>(nack)); });
}

void
Face::removePendingInterest(const PendingInterestId* pendingInterestId)
{
  m_ioService.post([=] { m_impl->asyncRemovePendingInterest(pendingInterestId); });
}

size_t
Face::getNPendingInterests() const
{
  return m_impl->m_pendingInterestTable.size();
}

const RegisteredPrefixId*
Face::setInterestFilter(const InterestFilter& interestFilter,
                  const OnInterest& onInterest,
                  const RegisterPrefixFailureCallback& onFailure,
                  const security::SigningInfo& signingInfo,
                  uint64_t flags)
{
    return setInterestFilter(interestFilter,
                             onInterest,
                             RegisterPrefixSuccessCallback(),
                             onFailure,
                             signingInfo,
                             flags);
}

const RegisteredPrefixId*
Face::setInterestFilter(const InterestFilter& interestFilter,
                  const OnInterest& onInterest,
                  const RegisterPrefixSuccessCallback& onSuccess,
                  const RegisterPrefixFailureCallback& onFailure,
                  const security::SigningInfo& signingInfo,
                  uint64_t flags)
{
    shared_ptr<InterestFilterRecord> filter =
      make_shared<InterestFilterRecord>(interestFilter, onInterest);

    nfd::CommandOptions options;
    options.setSigningInfo(signingInfo);

    return m_impl->registerPrefix(interestFilter.getPrefix(), filter,
                                  onSuccess, onFailure,
                                  flags, options);
}

const InterestFilterId*
Face::setInterestFilter(const InterestFilter& interestFilter,
                        const OnInterest& onInterest)
{
  shared_ptr<InterestFilterRecord> filter =
    make_shared<InterestFilterRecord>(interestFilter, onInterest);

  getIoService().post([=] { m_impl->asyncSetInterestFilter(filter); });

  return reinterpret_cast<const InterestFilterId*>(filter.get());
}

#ifdef NDN_FACE_KEEP_DEPRECATED_REGISTRATION_SIGNING

const RegisteredPrefixId*
Face::setInterestFilter(const InterestFilter& interestFilter,
                        const OnInterest& onInterest,
                        const RegisterPrefixSuccessCallback& onSuccess,
                        const RegisterPrefixFailureCallback& onFailure,
                        const IdentityCertificate& certificate,
                        uint64_t flags)
{
  security::SigningInfo signingInfo;
  if (!certificate.getName().empty()) {
    signingInfo = signingByCertificate(certificate.getName());
  }
  return setInterestFilter(interestFilter, onInterest,
                           onSuccess, onFailure,
                           signingInfo, flags);
}

const RegisteredPrefixId*
Face::setInterestFilter(const InterestFilter& interestFilter,
                        const OnInterest& onInterest,
                        const RegisterPrefixFailureCallback& onFailure,
                        const IdentityCertificate& certificate,
                        uint64_t flags)
{
  security::SigningInfo signingInfo;
  if (!certificate.getName().empty()) {
    signingInfo = signingByCertificate(certificate.getName());
  }
  return setInterestFilter(interestFilter, onInterest,
                             onFailure, signingInfo, flags);
}

const RegisteredPrefixId*
Face::setInterestFilter(const InterestFilter& interestFilter,
                        const OnInterest& onInterest,
                        const RegisterPrefixSuccessCallback& onSuccess,
                        const RegisterPrefixFailureCallback& onFailure,
                        const Name& identity,
                        uint64_t flags)
{
  security::SigningInfo signingInfo = signingByIdentity(identity);

  return setInterestFilter(interestFilter, onInterest,
                           onSuccess, onFailure,
                           signingInfo, flags);
}

const RegisteredPrefixId*
Face::setInterestFilter(const InterestFilter& interestFilter,
                        const OnInterest& onInterest,
                        const RegisterPrefixFailureCallback& onFailure,
                        const Name& identity,
                        uint64_t flags)
{
  security::SigningInfo signingInfo = signingByIdentity(identity);

  return setInterestFilter(interestFilter, onInterest,
                           onFailure, signingInfo, flags);
}

#endif // NDN_FACE_KEEP_DEPRECATED_REGISTRATION_SIGNING

const RegisteredPrefixId*
Face::registerPrefix(const Name& prefix,
               const RegisterPrefixSuccessCallback& onSuccess,
               const RegisterPrefixFailureCallback& onFailure,
               const security::SigningInfo& signingInfo,
               uint64_t flags)
{

    nfd::CommandOptions options;
    options.setSigningInfo(signingInfo);

    return m_impl->registerPrefix(prefix, shared_ptr<InterestFilterRecord>(),
                                  onSuccess, onFailure,
                                  flags, options);
}

#ifdef NDN_FACE_KEEP_DEPRECATED_REGISTRATION_SIGNING
const RegisteredPrefixId*
Face::registerPrefix(const Name& prefix,
                     const RegisterPrefixSuccessCallback& onSuccess,
                     const RegisterPrefixFailureCallback& onFailure,
                     const IdentityCertificate& certificate,
                     uint64_t flags)
{
  security::SigningInfo signingInfo;
  if (!certificate.getName().empty()) {
    signingInfo = signingByCertificate(certificate.getName());
  }
  return registerPrefix(prefix, onSuccess,
                        onFailure, signingInfo, flags);
}

const RegisteredPrefixId*
Face::registerPrefix(const Name& prefix,
                     const RegisterPrefixSuccessCallback& onSuccess,
                     const RegisterPrefixFailureCallback& onFailure,
                     const Name& identity,
                     uint64_t flags)
{
  security::SigningInfo signingInfo = signingByIdentity(identity);
  return registerPrefix(prefix, onSuccess,
                        onFailure, signingInfo, flags);
}
#endif // NDN_FACE_KEEP_DEPRECATED_REGISTRATION_SIGNING

void
Face::unsetInterestFilter(const RegisteredPrefixId* registeredPrefixId)
{
  m_ioService.post([=] { m_impl->asyncUnregisterPrefix(registeredPrefixId,
                                                       UnregisterPrefixSuccessCallback(),
                                                       UnregisterPrefixFailureCallback()); });
}

void
Face::unsetInterestFilter(const InterestFilterId* interestFilterId)
{
  m_ioService.post([=] { m_impl->asyncUnsetInterestFilter(interestFilterId); });
}

void
Face::unregisterPrefix(const RegisteredPrefixId* registeredPrefixId,
                       const UnregisterPrefixSuccessCallback& onSuccess,
                       const UnregisterPrefixFailureCallback& onFailure)
{
  m_ioService.post([=] { m_impl->asyncUnregisterPrefix(registeredPrefixId,onSuccess, onFailure); });
}

void
Face::processEvents(const time::milliseconds& timeout/* = time::milliseconds::zero()*/,
                    bool keepThread/* = false*/)
{
  if (m_ioService.stopped()) {
    m_ioService.reset(); // ensure that run()/poll() will do some work
  }

  try {
    if (timeout < time::milliseconds::zero()) {
        // do not block if timeout is negative, but process pending events
        m_ioService.poll();
        return;
      }

    if (timeout > time::milliseconds::zero()) {
      boost::asio::io_service& ioService = m_ioService;
      unique_ptr<boost::asio::io_service::work>& work = m_impl->m_ioServiceWork;
      m_impl->m_processEventsTimeoutEvent =
        m_impl->m_scheduler.scheduleEvent(timeout, [&ioService, &work] {
            ioService.stop();
            work.reset();
          });
    }

    if (keepThread) {
      // work will ensure that m_ioService is running until work object exists
      m_impl->m_ioServiceWork.reset(new boost::asio::io_service::work(m_ioService));
    }

    m_ioService.run();
  }
  catch (...) {
    m_impl->m_ioServiceWork.reset();
    m_impl->m_pendingInterestTable.clear();
    m_impl->m_registeredPrefixTable.clear();
    throw;
  }
}

void
Face::shutdown()
{
  m_ioService.post([this] { this->asyncShutdown(); });
}

void
Face::asyncShutdown()
{
  m_impl->m_pendingInterestTable.clear();
  m_impl->m_registeredPrefixTable.clear();

  if (m_transport->isConnected())
    m_transport->close();

  m_impl->m_ioServiceWork.reset();
}

/**
 * @brief extract local fields from NDNLPv2 packet and tag onto a network layer packet
 */
template<typename NETPKT>
static void
extractLpLocalFields(NETPKT& netPacket, const lp::Packet& lpPacket)
{
  if (lpPacket.has<lp::IncomingFaceIdField>()) {
    netPacket.getLocalControlHeader().
      setIncomingFaceId(lpPacket.get<lp::IncomingFaceIdField>());
  }
}
//onReceiveElement()这个动作的相应，实在数据包返回到端口的时候，
void
Face::onReceiveElement(const Block& blockFromDaemon)
{
  lp::Packet lpPacket(blockFromDaemon); // bare Interest/Data is a valid lp::Packet,
                                        // no need to distinguish

  Buffer::const_iterator begin, end;
  std::tie(begin, end) = lpPacket.get<lp::FragmentField>();
  Block netPacket(&*begin, std::distance(begin, end));

  switch (netPacket.type()) {
    case tlv::Interest: {
      shared_ptr<Interest> interest = make_shared<Interest>(netPacket);
      if (lpPacket.has<lp::NackField>()) {
        auto nack = make_shared<lp::Nack>(std::move(*interest));
        nack->setHeader(lpPacket.get<lp::NackField>());
        extractLpLocalFields(*nack, lpPacket);
        m_impl->nackPendingInterests(*nack);
      }
      else {
        extractLpLocalFields(*interest, lpPacket);
        m_impl->processInterestFilters(*interest);
      }
      break;
    }
    case tlv::Data: {
      //这里是获取底层返回的data包
      shared_ptr<Data> data = make_shared<Data>(netPacket);
      //添加face相关的操作
      extractLpLocalFields(*data, lpPacket);
      //返回到上层应用或则是转发到下一跳
      
      m_impl->satisfyPendingInterests(*data);

      //std::cout << "<------------onreceiveelement data---------------->" << std::endl;
      //std::cout << *data << std::endl;
      //change by jinyang
    /*  std::ofstream fout1("/home/ndn/Documents/mydir/12kbfromcomsumer.txt",std::ofstream::app);
      Block block1=(*data).getContent();
      for(Buffer::const_iterator  it=block1.value_begin(); it!=block1.value_end(); it++)
         {
               fout1<<*it;
           }
      fout1.close();*/
                        

    //  Face::m_store_face.push_back(data); //把所有底层应该获取的data的指针存储到一个数组中 

      /*
       *1、获得数据的名字和序号
       *    1）Name里面添加是用appendSegment，可以调用getSubName获取
       *2、重组内容
       *    1）根据第一步获取的序号重组返回的数据包
       *3、写入文件
       *    1）重组完成之后再写入文件
       */
      /*
        int m_totalSize=0;  //一个文件所聚合所有data包的字节数
        const Bloce& content =data.getContent();
        const Name& name=data.getName();
        //把所获得的数据写到输出流中
         std::cout.write(reinterpret_cast<const char*>(content.value() ), content.value_size() );
       m_total+=content.value.size();
       
       */
      //std::cout << "onReceiveElement test:\n" << *data << std::endl;
      break;
    }
  }
}

} // namespace ndn
