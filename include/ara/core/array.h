/**
 * Copytight (c) 2020
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
            using iterator = std::array<T, N>::iterator;
            using const_iterator = std::array<T, N>::const_iterator;
            using reverse_iterator = std::array<T, N>::reverse_iterator;
            using const_reverse_iterator = std::array<T, N>::const_reverse_iterator;

            Array() = default;

            template<typename... Args>
            constexpr Array(Args... args) : d_{args...}
            {
                static_assert(N == sizeof...(args));
            }

            /**
             * @brief Access specified element with bounds checking.
             * 
             * @param i position of the element to return.
             */
            constexpr T& at(int i) { return d_.at(i); }

            /**
             * @brief Access specified element with bounds checking.
             * 
             * @param i position of the element to return.
             */
            constexpr const T& at(int i) const { return d_.at(i); }
            
            /**
             * @brief Access specified element.
             * 
             * @param i position of the element to return.
             */ 
            constexpr T& operator [] (int i) { return d_[i]; }
            
            /**
             * @brief Access specified element.
             * 
             * @param i position of the element to return.
             */
            constexpr const T& operator [] (int i) const { return d_[i]; }

            /**
             * @brief Access the first element.
             * 
             */
            constexpr T& front() { return d_.front(); }

            /**
             * @brief Access the first element.
             * 
             */
            constexpr const T& front() const { return d_.front(); }

            /**
             * @brief Access the last element.
             * 
             */
            constexpr T& back() { return d_.back(); }

            /**
             * @brief Access the last element.
             * 
             */
            constexpr const T& back() const { return d_.back(); }

            /**
             * @brief Direct access to underlying array.
             * 
             */
            constexpr T* data() { return d_.data(); }

            /**
             * @brief Direct access to underlying array.
             * 
             */
            constexpr const T* data() const { return d_.data(); }

            /**
             * @brief Returns an iterator to the beginning.
             * 
             */
            constexpr iterator begin() { return d_.begin(); }

            /**
             * @brief Returns an iterator to the beginning.
             * 
             */
            constexpr const_iterator begin() const { return d_.begin(); }

            /**
             * @brief Returns an iterator to the beginning.
             * 
             */
            constexpr const_iterator cbegin() const { return d_.cbegin(); }

            /**
             * @brief Returns an iterator to the end.
             * 
             */
            constexpr iterator end() { return d_.end(); }

            /**
             * @brief Returns an iterator to the end.
             * 
             */
            constexpr const_iterator end() const { return d_.end(); }

            /**
             * @brief Returns an iterator to the end.
             * 
             */
            constexpr const_iterator cend() const { return d_.cend(); }

            /**
             * @brief Returns a reverse iterator to the beginning.
             * 
             */
            constexpr reverse_iterator rbegin() { return d_.rbegin(); }

            /**
             * @brief Returns a reverse iterator to the beginning.
             * 
             */
            constexpr const_reverse_iterator rbegin() const { return d_.rbegin(); }

            /**
             * @brief Returns a reverse iterator to the beginning.
             * 
             */
            constexpr const_reverse_iterator crbegin() const { return d_.crbegin(); }

            /**
             * @brief Returns a reverse iterator to the end.
             * 
             */
            constexpr reverse_iterator rend() { return d_.rend(); }

            /**
             * @brief Returns a reverse iterator to the end.
             * 
             */
            constexpr const_reverse_iterator rend() const { return d_.rend(); }

            /**
             * @brief Returns a reverse iterator to the end.
             * 
             */
            constexpr const_reverse_iterator crend() const { return d_.crend(); }

            /**
             * @brief Checks whether the container is empty.
             * 
             */
            [[nodiscard]] constexpr bool empty() const { return d_.empty(); }

            /**
             * @brief Returns the number of elements.
             * 
             */
            constexpr unsigned int size() const { return d_.size(); }

            /**
             * @brief Returns the maximum possible number of elements.
             * 
             */
            constexpr unsigned int max_size() const { return d_.max_size(); }

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
            constexpr void swap(Array<T,N>& other) { std::swap(d_, other.d_); }
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
