// **********************************************************************
//
// Copyright (c) 2003-2013 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.5.1
//
// <auto-generated>
//
// Generated from file `ObjectOracle.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef ____ObjectOracle_h__
#define ____ObjectOracle_h__

#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/Outgoing.h>
#include <Ice/OutgoingAsync.h>
#include <Ice/Incoming.h>
#include <Ice/Direct.h>
#include <IceUtil/ScopedArray.h>
#include <IceUtil/Optional.h>
#include <Ice/StreamF.h>
#include <Ice/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 305
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace RoboCompObjectOracle
{

class ObjectOracle;
void __read(::IceInternal::BasicStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompObjectOracle::ObjectOracle>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompObjectOracle::ObjectOracle*);

}

}

namespace RoboCompObjectOracle
{

class ObjectOracle;
bool operator==(const ObjectOracle&, const ObjectOracle&);
bool operator<(const ObjectOracle&, const ObjectOracle&);
::Ice::Object* upCast(::RoboCompObjectOracle::ObjectOracle*);
typedef ::IceInternal::Handle< ::RoboCompObjectOracle::ObjectOracle> ObjectOraclePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompObjectOracle::ObjectOracle> ObjectOraclePrx;
void __patch(ObjectOraclePtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompObjectOracle
{

struct ColorRGB
{
    ::Ice::Byte red;
    ::Ice::Byte green;
    ::Ice::Byte blue;

    bool operator==(const ColorRGB& __rhs) const
    {
        if(this == &__rhs)
        {
            return true;
        }
        if(red != __rhs.red)
        {
            return false;
        }
        if(green != __rhs.green)
        {
            return false;
        }
        if(blue != __rhs.blue)
        {
            return false;
        }
        return true;
    }

    bool operator<(const ColorRGB& __rhs) const
    {
        if(this == &__rhs)
        {
            return false;
        }
        if(red < __rhs.red)
        {
            return true;
        }
        else if(__rhs.red < red)
        {
            return false;
        }
        if(green < __rhs.green)
        {
            return true;
        }
        else if(__rhs.green < green)
        {
            return false;
        }
        if(blue < __rhs.blue)
        {
            return true;
        }
        else if(__rhs.blue < blue)
        {
            return false;
        }
        return false;
    }

    bool operator!=(const ColorRGB& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const ColorRGB& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const ColorRGB& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const ColorRGB& __rhs) const
    {
        return !operator<(__rhs);
    }
};

struct Label
{
    ::std::string name;
    ::Ice::Float believe;
};

typedef ::std::vector< ::RoboCompObjectOracle::ColorRGB> ColorSeq;

typedef ::std::vector< ::RoboCompObjectOracle::Label> ResultList;

}

namespace Ice
{
template<>
struct StreamableTraits< ::RoboCompObjectOracle::ColorRGB>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 3;
    static const bool fixedLength = true;
};

template<class S>
struct StreamWriter< ::RoboCompObjectOracle::ColorRGB, S>
{
    static void write(S* __os, const ::RoboCompObjectOracle::ColorRGB& v)
    {
        __os->write(v.red);
        __os->write(v.green);
        __os->write(v.blue);
    }
};

template<class S>
struct StreamReader< ::RoboCompObjectOracle::ColorRGB, S>
{
    static void read(S* __is, ::RoboCompObjectOracle::ColorRGB& v)
    {
        __is->read(v.red);
        __is->read(v.green);
        __is->read(v.blue);
    }
};

template<>
struct StreamableTraits< ::RoboCompObjectOracle::Label>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 5;
    static const bool fixedLength = false;
};

template<class S>
struct StreamWriter< ::RoboCompObjectOracle::Label, S>
{
    static void write(S* __os, const ::RoboCompObjectOracle::Label& v)
    {
        __os->write(v.name);
        __os->write(v.believe);
    }
};

template<class S>
struct StreamReader< ::RoboCompObjectOracle::Label, S>
{
    static void read(S* __is, ::RoboCompObjectOracle::Label& v)
    {
        __is->read(v.name);
        __is->read(v.believe);
    }
};

}

namespace RoboCompObjectOracle
{

class Callback_ObjectOracle_getLabelsFromImage_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_ObjectOracle_getLabelsFromImage_Base> Callback_ObjectOracle_getLabelsFromImagePtr;

}

namespace IceProxy
{

namespace RoboCompObjectOracle
{

class ObjectOracle : virtual public ::IceProxy::Ice::Object
{
public:

    void getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, ::RoboCompObjectOracle::ResultList& result)
    {
        getLabelsFromImage(image, result, 0);
    }
    void getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, ::RoboCompObjectOracle::ResultList& result, const ::Ice::Context& __ctx)
    {
        getLabelsFromImage(image, result, &__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::IceInternal::Function<void (const ::RoboCompObjectOracle::ResultList&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getLabelsFromImage(image, 0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getLabelsFromImage(image, 0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::RoboCompObjectOracle::ResultList&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getLabelsFromImage(image, &__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getLabelsFromImage(image, &__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::Ice::Context* __ctx, const ::IceInternal::Function<void (const ::RoboCompObjectOracle::ResultList&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent)
    {
        class Cpp11CB : public ::IceInternal::Cpp11FnCallbackNC
        {
        public:

            Cpp11CB(const ::std::function<void (const ::RoboCompObjectOracle::ResultList&)>& responseFunc, const ::std::function<void (const ::Ice::Exception&)>& exceptionFunc, const ::std::function<void (bool)>& sentFunc) :
                ::IceInternal::Cpp11FnCallbackNC(exceptionFunc, sentFunc),
                _response(responseFunc)
            {
                CallbackBase::checkCallback(true, responseFunc || exceptionFunc != nullptr);
            }

            virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
            {
                ::RoboCompObjectOracle::ObjectOraclePrx __proxy = ::RoboCompObjectOracle::ObjectOraclePrx::uncheckedCast(__result->getProxy());
                ::RoboCompObjectOracle::ResultList result;
                try
                {
                    __proxy->end_getLabelsFromImage(result, __result);
                }
                catch(::Ice::Exception& ex)
                {
                    Cpp11FnCallbackNC::__exception(__result, ex);
                    return;
                }
                if(_response != nullptr)
                {
                    _response(result);
                }
            }
        
        private:
            
            ::std::function<void (const ::RoboCompObjectOracle::ResultList&)> _response;
        };
        return begin_getLabelsFromImage(image, __ctx, new Cpp11CB(__response, __exception, __sent));
    }
    
public:
#endif

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image)
    {
        return begin_getLabelsFromImage(image, 0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::Ice::Context& __ctx)
    {
        return begin_getLabelsFromImage(image, &__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLabelsFromImage(image, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLabelsFromImage(image, &__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::RoboCompObjectOracle::Callback_ObjectOracle_getLabelsFromImagePtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLabelsFromImage(image, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq& image, const ::Ice::Context& __ctx, const ::RoboCompObjectOracle::Callback_ObjectOracle_getLabelsFromImagePtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLabelsFromImage(image, &__ctx, __del, __cookie);
    }

    void end_getLabelsFromImage(::RoboCompObjectOracle::ResultList& result, const ::Ice::AsyncResultPtr&);
    
private:

    void getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq&, ::RoboCompObjectOracle::ResultList&, const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq&, const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_adapterId(const ::std::string& __id) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_locatorCacheTimeout(int __timeout) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_connectionCached(bool __cached) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_secure(bool __secure) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_preferSecure(bool __preferSecure) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_collocationOptimized(bool __co) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_twoway() const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_twoway().get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_oneway() const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_oneway().get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_batchOneway() const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_datagram() const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_datagram().get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_batchDatagram() const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_compress(bool __compress) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_timeout(int __timeout) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_connectionId(const ::std::string& __id) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<ObjectOracle> ice_encodingVersion(const ::Ice::EncodingVersion& __v) const
    {
        return dynamic_cast<ObjectOracle*>(::IceProxy::Ice::Object::ice_encodingVersion(__v).get());
    }
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace IceDelegate
{

namespace RoboCompObjectOracle
{

class ObjectOracle : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq&, ::RoboCompObjectOracle::ResultList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&) = 0;
};

}

}

namespace IceDelegateM
{

namespace RoboCompObjectOracle
{

class ObjectOracle : virtual public ::IceDelegate::RoboCompObjectOracle::ObjectOracle,
                     virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq&, ::RoboCompObjectOracle::ResultList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace IceDelegateD
{

namespace RoboCompObjectOracle
{

class ObjectOracle : virtual public ::IceDelegate::RoboCompObjectOracle::ObjectOracle,
                     virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq&, ::RoboCompObjectOracle::ResultList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace RoboCompObjectOracle
{

class ObjectOracle : virtual public ::Ice::Object
{
public:

    typedef ObjectOraclePrx ProxyType;
    typedef ObjectOraclePtr PointerType;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void getLabelsFromImage(const ::RoboCompObjectOracle::ColorSeq&, ::RoboCompObjectOracle::ResultList&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getLabelsFromImage(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:
    virtual void __writeImpl(::IceInternal::BasicStream*) const;
    virtual void __readImpl(::IceInternal::BasicStream*);
    #ifdef __SUNPRO_CC
    using ::Ice::Object::__writeImpl;
    using ::Ice::Object::__readImpl;
    #endif
};

inline bool operator==(const ObjectOracle& l, const ObjectOracle& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

inline bool operator<(const ObjectOracle& l, const ObjectOracle& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

}

namespace RoboCompObjectOracle
{

template<class T>
class CallbackNC_ObjectOracle_getLabelsFromImage : public Callback_ObjectOracle_getLabelsFromImage_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompObjectOracle::ResultList&);

    CallbackNC_ObjectOracle_getLabelsFromImage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompObjectOracle::ObjectOraclePrx __proxy = ::RoboCompObjectOracle::ObjectOraclePrx::uncheckedCast(__result->getProxy());
        ::RoboCompObjectOracle::ResultList result;
        try
        {
            __proxy->end_getLabelsFromImage(result, __result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::CallbackNC<T>::callback.get()->*response)(result);
        }
    }

    Response response;
};

template<class T> Callback_ObjectOracle_getLabelsFromImagePtr
newCallback_ObjectOracle_getLabelsFromImage(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompObjectOracle::ResultList&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_ObjectOracle_getLabelsFromImage<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_ObjectOracle_getLabelsFromImagePtr
newCallback_ObjectOracle_getLabelsFromImage(T* instance, void (T::*cb)(const ::RoboCompObjectOracle::ResultList&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_ObjectOracle_getLabelsFromImage<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_ObjectOracle_getLabelsFromImage : public Callback_ObjectOracle_getLabelsFromImage_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompObjectOracle::ResultList&, const CT&);

    Callback_ObjectOracle_getLabelsFromImage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompObjectOracle::ObjectOraclePrx __proxy = ::RoboCompObjectOracle::ObjectOraclePrx::uncheckedCast(__result->getProxy());
        ::RoboCompObjectOracle::ResultList result;
        try
        {
            __proxy->end_getLabelsFromImage(result, __result);
        }
        catch(::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::__exception(__result, ex);
            return;
        }
        if(response)
        {
            (::IceInternal::Callback<T, CT>::callback.get()->*response)(result, CT::dynamicCast(__result->getCookie()));
        }
    }

    Response response;
};

template<class T, typename CT> Callback_ObjectOracle_getLabelsFromImagePtr
newCallback_ObjectOracle_getLabelsFromImage(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompObjectOracle::ResultList&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_ObjectOracle_getLabelsFromImage<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_ObjectOracle_getLabelsFromImagePtr
newCallback_ObjectOracle_getLabelsFromImage(T* instance, void (T::*cb)(const ::RoboCompObjectOracle::ResultList&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_ObjectOracle_getLabelsFromImage<T, CT>(instance, cb, excb, sentcb);
}

}

#endif
