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
// Generated from file `objectDetectionCNN.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef ____objectDetectionCNN_h__
#define ____objectDetectionCNN_h__

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

namespace RoboCompobjectDetectionCNN
{

class objectDetectionCNN;
void __read(::IceInternal::BasicStream*, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompobjectDetectionCNN::objectDetectionCNN>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RoboCompobjectDetectionCNN::objectDetectionCNN*);

}

}

namespace RoboCompobjectDetectionCNN
{

class objectDetectionCNN;
bool operator==(const objectDetectionCNN&, const objectDetectionCNN&);
bool operator<(const objectDetectionCNN&, const objectDetectionCNN&);
::Ice::Object* upCast(::RoboCompobjectDetectionCNN::objectDetectionCNN*);
typedef ::IceInternal::Handle< ::RoboCompobjectDetectionCNN::objectDetectionCNN> objectDetectionCNNPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompobjectDetectionCNN::objectDetectionCNN> objectDetectionCNNPrx;
void __patch(objectDetectionCNNPtr&, const ::Ice::ObjectPtr&);

}

namespace RoboCompobjectDetectionCNN
{

struct BoundingBox
{
    ::Ice::Float x;
    ::Ice::Float y;
    ::Ice::Float width;
    ::Ice::Float height;
};

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
    ::RoboCompobjectDetectionCNN::BoundingBox bb;
};

typedef ::std::vector< ::RoboCompobjectDetectionCNN::ColorRGB> ColorSeq;

typedef ::std::vector< ::RoboCompobjectDetectionCNN::Label> ResultList;

}

namespace Ice
{
template<>
struct StreamableTraits< ::RoboCompobjectDetectionCNN::BoundingBox>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 16;
    static const bool fixedLength = true;
};

template<class S>
struct StreamWriter< ::RoboCompobjectDetectionCNN::BoundingBox, S>
{
    static void write(S* __os, const ::RoboCompobjectDetectionCNN::BoundingBox& v)
    {
        __os->write(v.x);
        __os->write(v.y);
        __os->write(v.width);
        __os->write(v.height);
    }
};

template<class S>
struct StreamReader< ::RoboCompobjectDetectionCNN::BoundingBox, S>
{
    static void read(S* __is, ::RoboCompobjectDetectionCNN::BoundingBox& v)
    {
        __is->read(v.x);
        __is->read(v.y);
        __is->read(v.width);
        __is->read(v.height);
    }
};

template<>
struct StreamableTraits< ::RoboCompobjectDetectionCNN::ColorRGB>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 3;
    static const bool fixedLength = true;
};

template<class S>
struct StreamWriter< ::RoboCompobjectDetectionCNN::ColorRGB, S>
{
    static void write(S* __os, const ::RoboCompobjectDetectionCNN::ColorRGB& v)
    {
        __os->write(v.red);
        __os->write(v.green);
        __os->write(v.blue);
    }
};

template<class S>
struct StreamReader< ::RoboCompobjectDetectionCNN::ColorRGB, S>
{
    static void read(S* __is, ::RoboCompobjectDetectionCNN::ColorRGB& v)
    {
        __is->read(v.red);
        __is->read(v.green);
        __is->read(v.blue);
    }
};

template<>
struct StreamableTraits< ::RoboCompobjectDetectionCNN::Label>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 21;
    static const bool fixedLength = false;
};

template<class S>
struct StreamWriter< ::RoboCompobjectDetectionCNN::Label, S>
{
    static void write(S* __os, const ::RoboCompobjectDetectionCNN::Label& v)
    {
        __os->write(v.name);
        __os->write(v.believe);
        __os->write(v.bb);
    }
};

template<class S>
struct StreamReader< ::RoboCompobjectDetectionCNN::Label, S>
{
    static void read(S* __is, ::RoboCompobjectDetectionCNN::Label& v)
    {
        __is->read(v.name);
        __is->read(v.believe);
        __is->read(v.bb);
    }
};

}

namespace RoboCompobjectDetectionCNN
{

class Callback_objectDetectionCNN_getLabelsFromImage_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_objectDetectionCNN_getLabelsFromImage_Base> Callback_objectDetectionCNN_getLabelsFromImagePtr;

}

namespace IceProxy
{

namespace RoboCompobjectDetectionCNN
{

class objectDetectionCNN : virtual public ::IceProxy::Ice::Object
{
public:

    void getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, ::RoboCompobjectDetectionCNN::ResultList& result)
    {
        getLabelsFromImage(image, rows, cols, result, 0);
    }
    void getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, ::RoboCompobjectDetectionCNN::ResultList& result, const ::Ice::Context& __ctx)
    {
        getLabelsFromImage(image, rows, cols, result, &__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::IceInternal::Function<void (const ::RoboCompobjectDetectionCNN::ResultList&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getLabelsFromImage(image, rows, cols, 0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getLabelsFromImage(image, rows, cols, 0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::RoboCompobjectDetectionCNN::ResultList&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_getLabelsFromImage(image, rows, cols, &__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_getLabelsFromImage(image, rows, cols, &__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::Ice::Context* __ctx, const ::IceInternal::Function<void (const ::RoboCompobjectDetectionCNN::ResultList&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent)
    {
        class Cpp11CB : public ::IceInternal::Cpp11FnCallbackNC
        {
        public:

            Cpp11CB(const ::std::function<void (const ::RoboCompobjectDetectionCNN::ResultList&)>& responseFunc, const ::std::function<void (const ::Ice::Exception&)>& exceptionFunc, const ::std::function<void (bool)>& sentFunc) :
                ::IceInternal::Cpp11FnCallbackNC(exceptionFunc, sentFunc),
                _response(responseFunc)
            {
                CallbackBase::checkCallback(true, responseFunc || exceptionFunc != nullptr);
            }

            virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
            {
                ::RoboCompobjectDetectionCNN::objectDetectionCNNPrx __proxy = ::RoboCompobjectDetectionCNN::objectDetectionCNNPrx::uncheckedCast(__result->getProxy());
                ::RoboCompobjectDetectionCNN::ResultList result;
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
            
            ::std::function<void (const ::RoboCompobjectDetectionCNN::ResultList&)> _response;
        };
        return begin_getLabelsFromImage(image, rows, cols, __ctx, new Cpp11CB(__response, __exception, __sent));
    }
    
public:
#endif

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols)
    {
        return begin_getLabelsFromImage(image, rows, cols, 0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::Ice::Context& __ctx)
    {
        return begin_getLabelsFromImage(image, rows, cols, &__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLabelsFromImage(image, rows, cols, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLabelsFromImage(image, rows, cols, &__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::RoboCompobjectDetectionCNN::Callback_objectDetectionCNN_getLabelsFromImagePtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLabelsFromImage(image, rows, cols, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq& image, ::Ice::Int rows, ::Ice::Int cols, const ::Ice::Context& __ctx, const ::RoboCompobjectDetectionCNN::Callback_objectDetectionCNN_getLabelsFromImagePtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_getLabelsFromImage(image, rows, cols, &__ctx, __del, __cookie);
    }

    void end_getLabelsFromImage(::RoboCompobjectDetectionCNN::ResultList& result, const ::Ice::AsyncResultPtr&);
    
private:

    void getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq&, ::Ice::Int, ::Ice::Int, ::RoboCompobjectDetectionCNN::ResultList&, const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq&, ::Ice::Int, ::Ice::Int, const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_adapterId(const ::std::string& __id) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_locatorCacheTimeout(int __timeout) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_connectionCached(bool __cached) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_secure(bool __secure) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_preferSecure(bool __preferSecure) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_collocationOptimized(bool __co) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_twoway() const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_twoway().get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_oneway() const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_oneway().get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_batchOneway() const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_datagram() const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_datagram().get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_batchDatagram() const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_compress(bool __compress) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_timeout(int __timeout) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_connectionId(const ::std::string& __id) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<objectDetectionCNN> ice_encodingVersion(const ::Ice::EncodingVersion& __v) const
    {
        return dynamic_cast<objectDetectionCNN*>(::IceProxy::Ice::Object::ice_encodingVersion(__v).get());
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

namespace RoboCompobjectDetectionCNN
{

class objectDetectionCNN : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq&, ::Ice::Int, ::Ice::Int, ::RoboCompobjectDetectionCNN::ResultList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&) = 0;
};

}

}

namespace IceDelegateM
{

namespace RoboCompobjectDetectionCNN
{

class objectDetectionCNN : virtual public ::IceDelegate::RoboCompobjectDetectionCNN::objectDetectionCNN,
                           virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq&, ::Ice::Int, ::Ice::Int, ::RoboCompobjectDetectionCNN::ResultList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace IceDelegateD
{

namespace RoboCompobjectDetectionCNN
{

class objectDetectionCNN : virtual public ::IceDelegate::RoboCompobjectDetectionCNN::objectDetectionCNN,
                           virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq&, ::Ice::Int, ::Ice::Int, ::RoboCompobjectDetectionCNN::ResultList&, const ::Ice::Context*, ::IceInternal::InvocationObserver&);
};

}

}

namespace RoboCompobjectDetectionCNN
{

class objectDetectionCNN : virtual public ::Ice::Object
{
public:

    typedef objectDetectionCNNPrx ProxyType;
    typedef objectDetectionCNNPtr PointerType;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void getLabelsFromImage(const ::RoboCompobjectDetectionCNN::ColorSeq&, ::Ice::Int, ::Ice::Int, ::RoboCompobjectDetectionCNN::ResultList&, const ::Ice::Current& = ::Ice::Current()) = 0;
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

inline bool operator==(const objectDetectionCNN& l, const objectDetectionCNN& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

inline bool operator<(const objectDetectionCNN& l, const objectDetectionCNN& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

}

namespace RoboCompobjectDetectionCNN
{

template<class T>
class CallbackNC_objectDetectionCNN_getLabelsFromImage : public Callback_objectDetectionCNN_getLabelsFromImage_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RoboCompobjectDetectionCNN::ResultList&);

    CallbackNC_objectDetectionCNN_getLabelsFromImage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompobjectDetectionCNN::objectDetectionCNNPrx __proxy = ::RoboCompobjectDetectionCNN::objectDetectionCNNPrx::uncheckedCast(__result->getProxy());
        ::RoboCompobjectDetectionCNN::ResultList result;
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

template<class T> Callback_objectDetectionCNN_getLabelsFromImagePtr
newCallback_objectDetectionCNN_getLabelsFromImage(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompobjectDetectionCNN::ResultList&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_objectDetectionCNN_getLabelsFromImage<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_objectDetectionCNN_getLabelsFromImagePtr
newCallback_objectDetectionCNN_getLabelsFromImage(T* instance, void (T::*cb)(const ::RoboCompobjectDetectionCNN::ResultList&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_objectDetectionCNN_getLabelsFromImage<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_objectDetectionCNN_getLabelsFromImage : public Callback_objectDetectionCNN_getLabelsFromImage_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RoboCompobjectDetectionCNN::ResultList&, const CT&);

    Callback_objectDetectionCNN_getLabelsFromImage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), response(cb)
    {
    }

    virtual void __completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RoboCompobjectDetectionCNN::objectDetectionCNNPrx __proxy = ::RoboCompobjectDetectionCNN::objectDetectionCNNPrx::uncheckedCast(__result->getProxy());
        ::RoboCompobjectDetectionCNN::ResultList result;
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

template<class T, typename CT> Callback_objectDetectionCNN_getLabelsFromImagePtr
newCallback_objectDetectionCNN_getLabelsFromImage(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RoboCompobjectDetectionCNN::ResultList&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_objectDetectionCNN_getLabelsFromImage<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_objectDetectionCNN_getLabelsFromImagePtr
newCallback_objectDetectionCNN_getLabelsFromImage(T* instance, void (T::*cb)(const ::RoboCompobjectDetectionCNN::ResultList&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_objectDetectionCNN_getLabelsFromImage<T, CT>(instance, cb, excb, sentcb);
}

}

#endif
