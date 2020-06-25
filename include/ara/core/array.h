/**
 * Copyright (c) 2020
 * umlaut Software Development and contributors
 * 
 * SPDX-License-Identifier: MIT
 */

#ifndef ARA_CORE_ARRAY_H_
#define ARA_CORE_ARRAY_H_

#include <array> // std::array

namespace ara::core
{
    /**
     * @brief Container that encapsulates fixed size arrays.
     * 
     * @req {SWS_CORE_01201}
     */
    template <typename T, std::size_t N>
    class Array
    {
        public:
            // Draft only
            // todo: write our own iterators
            using iterator = std::array<T, N>::iterator;
            using const_iterator = std::array<T, N>::const_iterator;
            using reverse_iterator = std::array<T, N>::reverse_iterator;
            using const_reverse_iterator = std::array<T, N>::const_reverse_iterator;

            typedef T value_type;
            typedef std::size_t size_type;
            typedef std::ptrdiff_t difference_type;
            typedef value_type& reference;
            typedef const value_type& const_reference;
            typedef value_type* pointer;
            typedef const value_type* const_pointer;

            Array() = default;

            /**
             * @brief Initialization list constructor
             * 
             */
            template<typename... Args>
            constexpr Array(Args... args) : d_{args...}
            {
                static_assert(N == sizeof...(args), "Size is not the same");
            }

            /**
             * @brief Access specified element with bounds checking.
             * 
             * @param i position of the element to return.
             */
            constexpr reference at(size_type i) { return d_.at(i); }

            /**
             * @brief Access specified element with bounds checking.
             * 
             * @param i position of the element to return.
             */
            constexpr const_reference at(size_type i) const { return d_.at(i); }
            
            /**
             * @brief Access specified element.
             * 
             * @param i position of the element to return.
             */ 
            constexpr reference operator [] (size_type i) { return d_[i]; }
            
            /**
             * @brief Access specified element.
             * 
             * @param i position of the element to return.
             */
            constexpr const_reference operator [] (size_type i) const { return d_[i]; }

            /**
             * @brief Access the first element.
             * 
             */
            constexpr reference front() { return d_.front(); }

            /**
             * @brief Access the first element.
             * 
             */
            constexpr const_reference front() const { return d_.front(); }

            /**
             * @brief Access the last element.
             * 
             */
            constexpr reference back() { return d_.back(); }

            /**
             * @brief Access the last element.
             * 
             */
            constexpr const_reference back() const { return d_.back(); }

            /**
             * @brief Direct access to underlying array.
             * 
             */
            constexpr pointer data() noexcept { return d_.data(); }

            /**
             * @brief Direct access to underlying array.
             * 
             */
            constexpr const_pointer data() const noexcept { return d_.data(); }

            /**
             * @brief Returns an iterator to the beginning.
             * 
             */
            constexpr iterator begin() noexcept { return d_.begin(); }

            /**
             * @brief Returns an iterator to the beginning.
             * 
             */
            constexpr const_iterator begin() const noexcept { return d_.begin(); }

            /**
             * @brief Returns an iterator to the beginning.
             * 
             */
            constexpr const_iterator cbegin() const noexcept { return d_.cbegin(); }

            /**
             * @brief Returns an iterator to the end.
             * 
             */
            constexpr iterator end() noexcept { return d_.end(); }

            /**
             * @brief Returns an iterator to the end.
             * 
             */
            constexpr const_iterator end() const noexcept { return d_.end(); }

            /**
             * @brief Returns an iterator to the end.
             * 
             */
            constexpr const_iterator cend() const noexcept { return d_.cend(); }

            /**
             * @brief Returns a reverse iterator to the beginning.
             * 
             */
            constexpr reverse_iterator rbegin() noexcept { return d_.rbegin(); }

            /**
             * @brief Returns a reverse iterator to the beginning.
             * 
             */
            constexpr const_reverse_iterator rbegin() const noexcept { return d_.rbegin(); }

            /**
             * @brief Returns a reverse iterator to the beginning.
             * 
             */
            constexpr const_reverse_iterator crbegin() const noexcept { return d_.crbegin(); }

            /**
             * @brief Returns a reverse iterator to the end.
             * 
             */
            constexpr reverse_iterator rend() noexcept { return d_.rend(); }

            /**
             * @brief Returns a reverse iterator to the end.
             * 
             */
            constexpr const_reverse_iterator rend() const noexcept { return d_.rend(); }

            /**
             * @brief Returns a reverse iterator to the end.
             * 
             */
            constexpr const_reverse_iterator crend() const noexcept { return d_.crend(); }

            /**
             * @brief Checks whether the container is empty.
             * 
             */
            [[nodiscard]] constexpr bool empty() const noexcept { return d_.empty(); }

            /**
             * @brief Returns the number of elements.
             * 
             */
            constexpr size_type size() const noexcept { return d_.size(); }

            /**
             * @brief Returns the maximum possible number of elements.
             * 
             */
            constexpr size_type max_size() const noexcept { return d_.max_size(); }

            /**
             * @brief Fill the container with specified value.
             * 
             * @param value the value to assign to the elements.
             */
            constexpr void fill(const T& value) { d_.fill(value); }

            /**
             * @brief Swaps the contents.
             * 
             * @param other container to exchange the contents with.
             */
            constexpr void swap(Array<T,N>& other) noexcept(std::is_nothrow_swappable_v<T>) { std::swap(d_, other.d_); }
        private:
            std::array<T, N> d_;
    };

    /**
     * @brief Exchanges content between arrays.
     * 
     * @param lhs first argument of swap invocation.
     * @param rhs second argument of swap invocation.
     * 
     * @req {SWS_CORE_01296}
     */
    template <typename T, std::size_t N>
    void swap(Array<T, N> &lhs, Array<T, N> &rhs)
    {
        lhs.swap(rhs);
    }
} // namespace ara::core

#endif // ARA_CORE_ARRAY_H_
