function [text, anyErrors]=check_print(anyVar,anyHLM, anyErrors)


if (anyVar > anyHLM)
    anyErrors = [anyErrors,anyVar,','];
    ErrSign = '!';
else
    ErrSign = '.';
end

text = sprintf([' (',inputname(1),'= %4.3f)',ErrSign],anyVar);

