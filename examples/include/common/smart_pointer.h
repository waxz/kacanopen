//
// Created by waxz on 4/30/23.
//

#ifndef CMAKE_SUPER_BUILD_SMART_POINTER_H
#define CMAKE_SUPER_BUILD_SMART_POINTER_H

#include <memory>
#include <utility>
#include <type_traits>
#include <cstring> //strcmp

#include "type_info.h"

//-DDEFINE_ASSERT_MODE=AssertMode::THROW_ -DDEFINE_ASSERT_LEVEL=1
#define DEFINE_ASSERT_MODE AssertMode::THROW_
#define DEFINE_ASSERT_LEVEL 1
#include "dynamic_assert/assertion.h"


// right value reference template checking
// Because for forwarding reference, T will never be deduced as an rvalue reference.
// Suppose passing an object of type int to OwnershipReceiver, if the object is an lvalue, T will be deduced as an lvalue-reference, i.e. int&;
// if the object is an rvalue, T will be deduced as an non-reference, i.e. int.
// That's why std::is_rvalue_reference<T>::value won't work because it's always false.
//https://stackoverflow.com/questions/22654422/using-stdshared-ptrvoid-to-point-to-anything
//https://stackoverflow.com/questions/7863603/how-to-make-template-rvalue-reference-parameter-only-bind-to-rvalue-reference
//https://stackoverflow.com/questions/53758796/why-does-stdis-rvalue-reference-not-do-what-it-is-advertised-to-do


// memory and pointer
// raw pointer in stack cannot be manged by shared_ptr
// shared_ptr call free(pointer_on_stack) will fail
// https://shawnliu.me/post/creating-shared-ptr-from-raw-pointer-in-c++/


namespace common{
    class wild_ptr {

        std::shared_ptr<void> v;
        const char* type_name = nullptr;
    public:

        template <typename T>
        void set(const T& value) {
            std::cout << "******** copy set" << std::endl;

            type_name = TypeName<T>::Get();
            this->v = std::make_shared<T>(value);


        }

        template <typename T,typename = typename std::enable_if<!std::is_lvalue_reference<T>::value>::type>
        void set( T&& value) {
            std::cout << "******** move set" << std::endl;

            type_name = TypeName<T>::Get();
            this->v = std::make_shared<T>(std::forward<T>(value));

        }

        template <typename T,typename = typename std::enable_if<!std::is_pointer<T>::value>::type>
        void set( T* value) {
            std::cout << "******** pointer set" << std::endl;

            type_name = TypeName<T>::Get();
            this->v.reset(value);
        }

        template<typename T>
        void set( std::shared_ptr<T> value) {
            std::cout << "******** shared_ptr set" << std::endl;

            type_name = TypeName<T>::Get();
            this->v = value;
        }

        template<typename T, typename ...ARGS>
        void set( ARGS... args) {
            std::cout << "******** args... set" << std::endl;

            type_name = TypeName<T>::Get();
            this->v = std::make_shared<T>(std::forward<ARGS>(args)...);
        }

        template <typename T>
        T & ref() {
//            char buffer[500];
//            sprintf(buffer,"%s:%d, ref type_name = %s, allocated type_name = %s\n",__FILE__, __LINE__,TypeName<T>::Get(), type_name);
//            dynamic_assert(type_name != nullptr,buffer );
//            dynamic_assert(std::strcmp(type_name,TypeName<T>::Get() )==0,buffer );
            dynamic_assert(type_name != nullptr,"ref null" );
            dynamic_assert(std::strcmp(type_name,TypeName<T>::Get() )==0,"ref type not matched" );


            return (*(T*)v.get());
        }
    };

}
#endif //CMAKE_SUPER_BUILD_SMART_POINTER_H
