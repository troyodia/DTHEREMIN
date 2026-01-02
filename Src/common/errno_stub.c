int *__errno(void)
{
    static int err;
    return &err;
}