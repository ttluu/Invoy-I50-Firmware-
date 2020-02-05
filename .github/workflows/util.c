
float32 prompt_float()
{
  fprintf(STDOUT, "Value: ");
  float32 value = get_float();
  fprintf(STDOUT, "\r\nGot it: %.3f\r\n", value);

  return value;
}
