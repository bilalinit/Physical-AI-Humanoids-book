import React from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import CodeBlock from '@theme/CodeBlock';

type CodeExampleProps = {
  title?: string;
  description?: string;
  codeSnippets: {
    language: string;
    code: string;
    title: string;
  }[];
};

const CodeExample: React.FC<CodeExampleProps> = ({
  title = 'Code Example',
  description,
  codeSnippets
}) => {
  return (
    <div className="code-example">
      <h3>{title}</h3>
      {description && <p>{description}</p>}

      <Tabs>
        {codeSnippets.map((snippet, index) => (
          <TabItem key={index} value={snippet.language} label={snippet.title}>
            <CodeBlock language={snippet.language} title={snippet.title}>
              {snippet.code}
            </CodeBlock>
          </TabItem>
        ))}
      </Tabs>
    </div>
  );
};

export default CodeExample;