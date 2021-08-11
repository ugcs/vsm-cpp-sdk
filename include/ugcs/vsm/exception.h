// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file exception.h
 *
 * VSM exceptions definition.
 */

#ifndef _UGCS_VSM_EXCEPTION_H_
#define _UGCS_VSM_EXCEPTION_H_

#include <ugcs/vsm/defs.h>
#include <stdarg.h>
#include <string>

namespace ugcs {
namespace vsm {

/** Base class for all VSM exceptions. */
class Exception: public std::exception {
public:
    /** Dummy structure to explicitly indicate the constructor
     * overload for va_list type argument. This is to workaround
     * the problem in Windows, where va_list has 'char*' type and
     * automatic overload resolution is confused between real va_list
     * argument and constant string literal.
     */
    struct Va_list_overload {
        // Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=60336
        char dummy;
    };

    /** Dummy structure to explicitly indicate the constructor
     * overload for variable arguments (i.e. printf style).
     */
    struct Va_args_overload {
        // Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=60336
        char dummy;
    };

    /** Default constructor should not be used often. */
    Exception(): msg("<no message>") {}

    /** Construct exception.
     *
     * @param msg Specified formatted description.
     */
    Exception(Va_args_overload, const char *msg, ...) __FORMAT(printf, 3, 4);

    /** Construct exception.
     *
     * @param msg Specified formatted description.
     * @param args Description format parameters pack.
     */
    Exception(Va_list_overload, const char *msg, va_list args) __FORMAT(printf, 3, 0);

    virtual
    ~Exception() noexcept
    {}

    /** Get readable exception description. */
    virtual const char *
    what() const noexcept override
    {
        return msg.c_str();
    }

protected:
    /** Exception message. */
    std::string msg;

    /** Create message string (data member "msg") from format parameters. */
    void
    Create_msg(const char *msg, va_list args);
};

/** Exception class with one parameter. If you need more than one parameter then
 * wrap them into class and specify the class here.
 *
 * @param TParam Type for exception parameter.
 * @param Dummy Type used for differentiating exception types signatures.
 */
template <typename Dummy = void, typename TParam = int>
class Param_exception: public Exception {
public:
    /** User-defined parameter. */
    TParam param;

    // @{
    /** Different constructors for optionally taking the parameter value
     * and formatted message.
     */
    Param_exception(Va_args_overload, const char *msg, ...) __FORMAT(printf, 3, 4)
    {
        va_list args;
        va_start(args, msg);
        Create_msg(msg, args);
        va_end(args);
    }

    Param_exception(Va_list_overload, const char *msg, va_list args) __FORMAT(printf, 3, 0)
    {
        Create_msg(msg, args);
    }

    Param_exception(Va_args_overload, const TParam &param, const char *msg, ...)
        __FORMAT(printf, 4, 5):
        param(param)
    {
        va_list args;
        va_start(args, msg);
        Create_msg(msg, args);
        va_end(args);
    }

    Param_exception(Va_list_overload, const TParam &param, const char *msg, va_list args)
        __FORMAT(printf, 4, 0):
        param(param)
    {
        Create_msg(msg, args);
    }
    // @}
};

/** Helper class for defining derived exceptions. It automatically forwards
 * constructor call to base class.
 * @param Base_exception
 * @param Dummy Type used for differentiating exception types signatures.
 */
template <class Base_exception, typename Dummy = void>
class Derived_exception: public Base_exception {
public:
    /** Forwarding constructor. */
    template <typename... Args>
    Derived_exception(Args &&... args):
        Base_exception(std::forward<Args>(args)...)
    {}
};

#ifdef DEBUG

/** Throw VSM exception instance.
 * @param __exc_class Exception class name.
 * @param __msg Formatted description message.
 * @param ... Message format parameters.
 */
#define VSM_EXCEPTION(__exc_class, __msg, ...) \
    throw __exc_class(__exc_class::Va_args_overload(), "[%s:%d] " __msg, __FILE__, __LINE__, ## __VA_ARGS__)

/** Throw VSM exception instance with parameter.
 * @param __exc_class Exception class name.
 * @param __param Parameter value.
 * @param __msg Formatted description message.
 * @param ... Message format parameters.
 */
#define VSM_PARAM_EXCEPTION(__exc_class, __param, __msg, ...) \
    throw __exc_class(__exc_class::Va_args_overload(), __param, "[%s:%d] " __msg, __FILE__, __LINE__, ## __VA_ARGS__)

#else /* DEBUG */

/** Throw VSM exception instance.
 * @param __exc_class Exception class name.
 * @param __msg Formatted description message.
 * @param ... Message format parameters.
 */
#define VSM_EXCEPTION(__exc_class, __msg, ...) \
    /* Dummy format argument is added to avoid GCC 4.8.1 bug related to printf attribute. */ \
    throw __exc_class(__exc_class::Va_args_overload(), "%s" __msg, "", ## __VA_ARGS__)

/** Throw VSM exception instance with parameter.
 * @param __exc_class Exception class name.
 * @param __param Parameter value.
 * @param __msg Formatted description message.
 * @param ... Message format parameters.
 */
#define VSM_PARAM_EXCEPTION(__exc_class, __param, __msg, ...) \
    /* Dummy format argument is added to avoid GCC 4.8.1 bug related to printf attribute. */ \
    throw __exc_class(__exc_class::Va_args_overload(), __param, "%s" __msg, "", ## __VA_ARGS__)

#endif /* DEBUG */

/** Throw VSM system exception - i.e. the one caused by system call failure.
 * Special handling is required because system error code might be overwritten
 * by exception allocation functions, so it must be taken as soon as possible.
 * @param __msg Descriptive message of the failure context.
 * @param ... Message format parameters.
 */
#define VSM_SYS_EXCEPTION(__msg, ...) do { \
    std::string sys_msg = ugcs::vsm::Log::Get_system_error(); \
    VSM_EXCEPTION(ugcs::vsm::System_exception, __msg ": %s", ## __VA_ARGS__, sys_msg.c_str()); \
} while (false)

/** Define custom exception type.
 * @param __exc_class Name for new exception type.
 * @param ... Parameter type can be specified as second argument.
 * @see Param_exception
 */
#define VSM_DEFINE_EXCEPTION(__exc_class, ...) \
    struct __exc_class ## _dummy_struct {}; \
    typedef ugcs::vsm::Param_exception<__exc_class ## _dummy_struct, ## __VA_ARGS__> __exc_class

/** Define custom derived exception.
 * @param __base_class Type of base exception class.
 * @param __exc_class Name for new exception type.
 */
#define VSM_DEFINE_DERIVED_EXCEPTION(__base_class, __exc_class) \
    /** Type for a new exception class differentiation. */ \
    struct __exc_class ## _dummy_struct {}; \
    /** Definition of a new exception type. */ \
    typedef ugcs::vsm::Derived_exception<__base_class, __exc_class ## _dummy_struct> __exc_class


/* Some common exceptions below. */

/** Exception to throw when debugging assertion fires. */
VSM_DEFINE_EXCEPTION(Debug_assert_exception);
/** Indicates that some object is not longer exists while its service is invoked. */
VSM_DEFINE_EXCEPTION(Nullptr_exception);
/** Indicates that some invalid parameter was passed to SDK API call. */
VSM_DEFINE_EXCEPTION(Invalid_param_exception);
/** Indicates that the operation is invalid in current state. */
VSM_DEFINE_EXCEPTION(Invalid_op_exception);
/** Some unexpected internal error occurred. Potential SW bug. */
VSM_DEFINE_EXCEPTION(Internal_error_exception);
/** Exception for system call failure. */
VSM_DEFINE_EXCEPTION(System_exception);
/** Altimeter required exception. For certain mission types */
VSM_DEFINE_EXCEPTION(Altimeter_required_exception);

} /* namespace vsm */
} /* namespace ugcs */

#endif /* _UGCS_VSM_EXCEPTION_H_ */
