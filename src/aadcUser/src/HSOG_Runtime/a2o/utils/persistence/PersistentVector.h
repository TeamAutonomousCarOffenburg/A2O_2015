/*
 * PersistentVector.h
 *
 *  Created on: 20/12/2011
 *      Author: ignacio
 */
#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>


class VectorException {};

template <typename T>
class PersistentVector{
	public:
		class Node {
			public:
				virtual ~Node() {};
		};

		class Leaf :public Node{
			T value;
			public:
			Leaf(T newvalue) :value(newvalue){};
			Leaf (){};
			virtual ~Leaf() {};
			T getValue() {return value;};
		};
		typedef boost::shared_ptr<Node > Node_ptr;
		typedef std::vector<Node_ptr> Node_vector;

		class Branch : public Node{
			Node_vector array;
			public:
			Branch(Node_vector &other):
				array(other) {};
			Branch() {};
			virtual ~Branch() {};
			Node_vector getArray() {return array;};
		};
		unsigned int _cnt;
		unsigned int _shift;
		Node_ptr _root;
		Node_vector _tail;

	public:
		typedef boost::shared_ptr<PersistentVector<T>> Ptr;
		typedef boost::shared_ptr<const PersistentVector<T>> ConstPtr;
		
		PersistentVector<T>() :
			_cnt(0), _shift(5),_root(new Branch()) {}

		PersistentVector<T>(unsigned int cnt, unsigned int shift, Node_ptr root, Node_vector &tail):
			_cnt(cnt),_shift(shift),_root(root),_tail(tail) {}

		unsigned int tailoff() const{
			if(_cnt < 32)
				return 0;
			return ((_cnt - 1) >> 5) << 5;
		};


		unsigned int size() const {return _cnt;}

		T get(unsigned int i) const {
			if(i >= 0 && i < _cnt)
			{
				Node_vector array;
				if(i < tailoff()) {
					Node_ptr node = _root;
					for(int level = _shift; level > 0; level -= 5)
						node = dynamic_cast<Branch *> (node.get())->getArray()[(i >> level) & 0x01f];
					array=dynamic_cast<Branch *> (node.get())->getArray();
				}
				else
					array=_tail;
				return dynamic_cast<Leaf *> (array[i & 0x01f].get())->getValue();
			}
			throw VectorException();
		};

		PersistentVector<T> put(unsigned int pos, const T &val){
			if(pos >= 0 && pos < _cnt)
			{
				PersistentVector<T> newVector(*this);
				if(pos >= tailoff())
				{
					newVector._tail[pos & 0x01f]=Node_ptr(new Leaf(val));
					return newVector;
				}
				newVector._root=doPut(_shift,_root,pos,val);
				return newVector;
			}
			if(pos == _cnt)
				return add(val);
			throw VectorException();
		}

		Node_ptr doPut(unsigned int level, Node_ptr node,unsigned int pos, const T &val) {
			Node_vector array= dynamic_cast<Branch *>(node.get())->getArray();
			if(level == 0)
			{
				array[pos & 0x01f] = Node_ptr(new Leaf(val));
			}
			else
			{
				int subidx = (pos >> level) & 0x01f;
				array[subidx] = doPut(level-5, array[subidx], pos, val);
			}
			return Node_ptr(new Branch(array));
		}

		PersistentVector<T> add(const T &val){
			//room in tail?
			// if(tail.length < 32)
			if(_cnt - tailoff() < 32)
			{
				Node_vector newTail=_tail;
				newTail.push_back(Node_ptr(new Leaf(val)));
				return PersistentVector(_cnt + 1, _shift, _root, newTail);
			}
			//full tail, push into tree
			PersistentVector<T> ret=PersistentVector<T>();
			ret._cnt=_cnt+1;
			ret._shift=_shift;
			ret._tail.push_back(Node_ptr(new Leaf(val)));
			Node_ptr tailnode(new Branch(_tail));
			//overflow root?
			if((_cnt >>  5) > ((unsigned)1 << _shift))
			{
				Node_vector array;
				array.push_back(_root);
				array.push_back(createPath(_shift, tailnode));
				ret._root=Node_ptr(new Branch(array));
				ret._shift += 5;
			}
			else
			{
				ret._root=pushTail(_shift, _root, tailnode);
			}
			return ret;
		}


		Node_ptr createPath(unsigned int level, Node_ptr &node) {
			if (level==0)
				return node;
			Node_vector array;
			array.push_back(node);
			Node_ptr new_node(new Branch(array));
			return createPath(level-5,new_node);
		};

		Node_ptr pushTail(unsigned int level, Node_ptr &parent, Node_ptr &tailnode){
			unsigned int subidx = ((_cnt - 1) >> level) & 0x01f;
			Branch *parent_branch= dynamic_cast<Branch *> (parent.get());
			Node_vector array = parent_branch->getArray();
			Node_ptr newNode;
			if(level == 5)
			{
				newNode=tailnode;
			}
			else
			{
				if (array.size() > subidx) {
					Node_ptr child= parent_branch->getArray()[subidx];
					newNode=pushTail(level-5,child, tailnode);
				}
				else
					newNode=createPath(level-5, tailnode);
			}
			if (array.size()>subidx)
				array[subidx]=newNode;
			else
				array.push_back(newNode);
			return Node_ptr(new Branch(array));
		};

		PersistentVector<T> pop() {
			if (_cnt == 0)
				throw VectorException();
			if (_cnt == 1)
				return PersistentVector<T>();
			if (_tail.size() > 1)
			{
				PersistentVector<T> newVector (*this);
				newVector._cnt=_cnt-1;
				newVector._tail.pop_back();
				return newVector;
			}

			PersistentVector<T> newVector;
			newVector._cnt=_cnt-1;
			newVector._tail=findTail();
			newVector._root=popRoot(_shift,_root);
			newVector._shift=_shift;
			Branch * rootBranch=static_cast<Branch *> (newVector._root.get());
			if(_shift > 5 && rootBranch->getArray().size()==1)
			{
				newVector._root=rootBranch->getArray()[0];
				newVector._shift -= 5;
			}
			return newVector;
		}

		Node_vector findTail() {
			unsigned int pos = _cnt-2;
			Node_ptr node=_root;
			for (int i=_shift; i>0; i-=5)
				node=dynamic_cast<Branch *> (node.get())->getArray()[(pos >> i) & 0x01f];
			return dynamic_cast<Branch *> (node.get())->getArray();
		};

		Node_ptr popRoot(unsigned int level,Node_ptr &node) {
			unsigned int pos = _cnt-2;
			unsigned int subidx = (pos >> level) & 0x01f;
			Node_vector array = static_cast<Branch *> (node.get())->getArray();
			if (level > 5) {
				Node_ptr newNode=popRoot(level-5,array[subidx]);
				if (static_cast<Branch *> (newNode.get())->getArray().size() == 0)
					array.pop_back();
			}
			else
				array.pop_back();

			return Node_ptr(new Branch(array));
		};
};
