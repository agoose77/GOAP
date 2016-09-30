from collections.abc import Sized, Mapping


class ListView(Sized):
    def __init__(self, list_):
        self._list = list_

    def __getitem__(self, item):
        return self._list[item]

    def __len__(self):
        return len(self._list)

    def __repr__(self):
        return repr(self._list)


class DictView(Mapping):
    def __init__(self, dict_):
        self._dict = dict_

    def __getitem__(self, index):
        return self._dict[index]

    def __iter__(self):
        return iter(self._dict)

    def __len__(self):
        return len(self._dict)

    def __repr__(self):
        return repr(self._dict)
