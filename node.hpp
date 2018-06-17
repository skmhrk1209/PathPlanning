#pragma once

#include <memory>
#include <vector>

template <typename State, typename Value>
class Node
{
public:

	using Ptr = std::shared_ptr<Node<State, Value>>;
	using WeakPtr = std::weak_ptr<Node<State, Value>>;

	template <typename _State, typename _Value>
	Node(_State&& state, _Value&& value) :
		mState(std::forward<_State>(state)), mValue(std::forward<_Value>(value)) {}

	template <typename _Ptr, typename _State, typename _Value>
	Node(_Ptr&& parent, _State&& state, _Value&& value) :
		mParent(std::forward<_Ptr>(parent)), mState(std::forward<_State>(state)), mValue(std::forward<_Value>(value)) {}

	template <typename _State, typename _Value>
	static Ptr create(_State&& state, _Value&& value)
	{
		return std::make_shared<Node<State, Value>>(std::forward<_State>(state), std::forward<_Value>(value));
	}

	template <typename _Ptr, typename _State, typename _Value>
	static Ptr create(_Ptr&& parent, _State&& state, _Value&& value)
	{
		return std::make_shared<Node<State, Value>>(std::forward<_Ptr>(parent), std::forward<_State>(state), std::forward<_Value>(value));
	}

	const WeakPtr& parent() const { return mParent; }
	WeakPtr& parent() { return mParent; }

	const std::vector<Ptr>& children() const { return mChildren; }
	std::vector<Ptr>& children() { return mChildren; }

	const State& state() const { return mState; }
	State& state() { return mState; }

	const Value& value() const { return mValue; }
	Value& value() { return mValue; }

private:

	WeakPtr mParent;
	std::vector<Ptr> mChildren;

	State mState;
	Value mValue;
};