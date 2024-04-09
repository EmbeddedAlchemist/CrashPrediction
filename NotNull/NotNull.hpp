#pragma once

template <typename Type>
Type& NotNullRef(Type* pointer) {
	assert(pointer != nullptr);
	return *pointer;
}

template <typename Type>
Type& NotNullRef(Type& ref) {
	assert(&ref != nullptr);
	return ref;
}


template <typename Type>
Type* NotNullPtr(Type* ptr) {
	assert(ptr != nullptr);
	return ptr;

}

template <typename Type>
Type* NotNullPtr(Type& ref) {
	assert(&ref != nullptr);
	return &ref;

}

template <typename Type>
Type& NotNull(Type& ref) {
	assert(&ref != nullptr);
	return ref;
}

template <typename Type>
Type* NotNull(Type* ref) {
	assert(ref != nullptr);
	return ref;
}


