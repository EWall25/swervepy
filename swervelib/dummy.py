class Dummy:
    def return_zero(self, *_args, **_kwargs):
        return 0

    def __getattr__(self, _):
        return self.return_zero
