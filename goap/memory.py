from collections import OrderedDict
from inspect import Signature, Parameter
from operator import attrgetter


class EmptyValue:
    pass


def _generate_record_class(*field_names):
    name_to_getter = OrderedDict(((n, attrgetter(n)) for n in field_names))

    cls_dict = {}

    def record_repr(self):
        as_str = ", ".join(("{}={}".format(n, f(self)) for n, f in self.field_getters))
        return "Record({})".format(as_str)

    parameters = [Parameter(n, Parameter.POSITIONAL_OR_KEYWORD, default=EmptyValue) for n in field_names]
    signature = Signature(parameters)

    def record_init(self, *args, **kwargs):
        data = signature.bind(*args, **kwargs)
        data.apply_defaults()

        self.__dict__.update(data.arguments.items())
        self.__dict__["field_getters"] = [
            (n, g) for n, g in name_to_getter.items() if data.arguments[n] is not EmptyValue
        ]

    def record_setattr(self, name, value):
        raise RuntimeError("Attributes of record read-only")

    cls_dict["__repr__"] = record_repr
    cls_dict["__init__"] = record_init
    cls_dict["__setattr__"] = record_setattr

    return type("Record", (), cls_dict)


Record = _generate_record_class("type", "subject", "target", "confidence", "data", "update_time")


class RecordMemory:
    def __init__(self):
        self._records = []

    def add_record(self, record):
        self._records.append(record)

    def query(self, query_record):
        matches = [(g, g(query_record)) for n, g in query_record.field_getters]

        for record in self._records:
            for getter, value in matches:
                if getter(record) != value:
                    break
            else:
                yield record

    def count_query(self, query_record):
        return len(list(self.query(query_record)))


if __name__ == "__main__":
    memory = RecordMemory()

    from collections import namedtuple

    obj_cls = namedtuple("SomeObj", "name")
    some_obj = obj_cls("Obj")
    some_other_obj = obj_cls("OtherObj")

    rec1 = Record("crouching", some_obj)
    memory.add_record(rec1)

    memory.add_record(Record("crouching", some_other_obj))
    memory.add_record(Record("attacking", some_obj, some_other_obj))

    assert tuple(memory.query(Record("crouching", some_obj))) == (rec1,)
